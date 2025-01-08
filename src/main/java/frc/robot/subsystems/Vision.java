// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Utils.Constants;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {

  // Cameras
  // Do not reference directly
  private PhotonCamera aprilTagCamera;
  //private PhotonCamera objectDetectionCamera;

  // Pipeline Results
  // These results are updated in the periodic
  // All methods should reference these results
  private PhotonPipelineResult aprilTagResult;
  //private PhotonPipelineResult objectResult;
  private boolean aprilTagDetected;
  private boolean objectDetected;

  // Pose Estimation
  private PhotonPoseEstimator visionEstimator;
  private Matrix<N3, N1> curStdDevs;

  // Simulation
  private PhotonCameraSim aprilTagSim;
  //private PhotonCameraSim objectDetectionSim;
  private VisionSystemSim visionSim;


  /** Creates a new Vision. */
  public Vision() {
    aprilTagCamera = new PhotonCamera("April_Tag_Camera");
    //objectDetectionCamera = new PhotonCamera("Object_Detection_Camera");

    
    visionEstimator = new PhotonPoseEstimator(Constants.kTagLayout , PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.kRobotToCam);
    visionEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    // ----- Simulation
    if (Robot.isSimulation()) {
        // Create the vision system simulation which handles cameras and targets on the field.
        visionSim = new VisionSystemSim("main");
        // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
        visionSim.addAprilTags(Constants.kTagLayout);
        // Create simulated camera properties. These can be set to mimic your actual camera.
        var cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
        cameraProp.setCalibError(0.35, 0.10);
        cameraProp.setFPS(15);
        cameraProp.setAvgLatencyMs(50);
        cameraProp.setLatencyStdDevMs(15);
        // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
        // targets.
        aprilTagSim = new PhotonCameraSim(aprilTagCamera, cameraProp);
        //objectDetectionSim = new PhotonCameraSim(objectDetectionCamera);
        // Add the simulated camera to view the targets on this simulated field.
        visionSim.addCamera(aprilTagSim, Constants.kRobotToCam);

        aprilTagSim.enableDrawWireframe(true);
    }
  }


      /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop.
     *
     * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
     * {@link getEstimationStdDevs}
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
     *     used for estimation.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        if (aprilTagDetected) {
            visionEst = visionEstimator.update(aprilTagResult);
            updateEstimationStdDevs(visionEst, aprilTagResult.getTargets());

            // ----- Simulation

            if (Robot.isSimulation()) {
                visionEst.ifPresentOrElse(
                        est ->
                                getSimDebugField()
                                        .getObject("VisionEstimation")
                                        .setPose(est.estimatedPose.toPose2d()),
                        () -> {
                            getSimDebugField().getObject("VisionEstimation").setPoses();
                        });
            }
        }
        return visionEst;
    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     */
    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = Constants.kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = Constants.kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = visionEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = Constants.kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = Constants.kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }

    /**
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //Update April Tag Results
    aprilTagDetected = false;
    var tempTagResults = aprilTagCamera.getAllUnreadResults();
    if (!tempTagResults.isEmpty()) {
      // Camera processed a new frame since last
      // Get the last one in the list.
      aprilTagResult = tempTagResults.get(tempTagResults.size() - 1);
      if (aprilTagResult.hasTargets()) {
          // At least one AprilTag was seen by the camera
          aprilTagDetected = true;
      }
    }
    
    // //Update Object Results
    // objectDetected = false;
    // var tempObjectResults = objectDetectionCamera.getAllUnreadResults();
    // if (!tempObjectResults.isEmpty()) {
    //   // Camera processed a new frame since last
    //   // Get the last one in the list.
    //   objectResult = tempObjectResults.get(tempObjectResults.size() - 1);
    //   if (objectResult.hasTargets()) {
    //       // At least one AprilTag was seen by the camera
    //       objectDetected = true;
    //   }
    // }

    var pose = getEstimatedGlobalPose();
    pose.ifPresent(
      est -> {
        // Change our trust in the measurement based on the tags we can see
        var estStdDevs = getEstimationStdDevs();

        SubsystemsInst.getInst().drivetrain.addVisionMeasurement(
                est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
      });
  }



  // ----- Simulation

  public void simulationPeriodic(Pose2d robotSimPose) {
      visionSim.update(robotSimPose);
  }

  /** Reset pose history of the robot in the vision system simulation. */
  public void resetSimPose(Pose2d pose) {
      if (Robot.isSimulation()) visionSim.resetRobotPose(pose);
  }

  /** A Field2d for visualizing our robot and objects on the field. */
  public Field2d getSimDebugField() {
      if (!Robot.isSimulation()) return null;
      return visionSim.getDebugField();
  }
}
