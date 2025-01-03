// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IO;
import frc.robot.Utils.Constants;
import frc.robot.Utils.Utils;
import frc.robot.lib.SdsSwerveModule;
import frc.robot.lib.controls.XBoxController;



/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
  public boolean fieldRelative;

  private double limiter;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(10);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(10);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(10);

  private final SdsSwerveModule m_frontLeft = new SdsSwerveModule(5, 6);
  private final SdsSwerveModule m_backLeft = new SdsSwerveModule(7, 8);
  private final SdsSwerveModule m_frontRight = new SdsSwerveModule(3, 4);
  private final SdsSwerveModule m_backRight = new SdsSwerveModule(1, 2);
  private XBoxController driveController;
//first two colums above are done
  private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI, AHRS.NavXUpdateRate.k50Hz);


  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          Constants.m_kinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          });

  public Drivetrain() {
    m_gyro.reset();
    fieldRelative = true;
    driveController = IO.getInstance().getDriverController();
  }


  /**
   * Set up path planner after subsystem is initialized in order to avoid singleton recursion.
   */
  public void setupPathPlanner(){
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
      config = Constants.autoConfig;
    }
    //m_gyro.reset();
      // Configure the AutoBuilder last
    AutoBuilder.configure(
      this::getPose, // Robot pose supplier
      this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
      new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
          new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
          new PIDConstants(4.0, 0.0, 0.0) // Rotation PID constants
          // 4.0, // Max module speed, in m/s
          // Constants.kDriveRadius, // Drive base radius in meters. Distance from robot center to furthest module.
          // new ReplanningConfig() // Default path replanning config. See the API for the options here
      ),
      config,
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this // Reference to this subsystem to set requirements
    );
  }


  /**
   * Generic method to drive the robot using speed inputs
   *
   * @param xSpeed m/s speed of the robot in the x direction (forward).
   * @param ySpeed m/s speed of the robot in the y direction (sideways).
   * @param rot rad/sec angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    ChassisSpeeds chassisSpeeds = fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(-m_gyro.getFusedHeading()))
                : new ChassisSpeeds(xSpeed, ySpeed, rot);
    //Time slice discretization code taken from 254/YAGSL
    //Compensates for second order kinematic drift
    ChassisSpeeds.discretize(chassisSpeeds, 0.02); 
    var swerveModuleStates =
        Constants.m_kinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }


  /**
   * Used by pathplanner to drive using robot relative inputs.
   * @param chassisSpeeds the commanded chassisSpeeds to drive at
   */
  public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
    chassisSpeeds.omegaRadiansPerSecond = -chassisSpeeds.omegaRadiansPerSecond;

    //Time slice discretization code taken from 254/YAGSL
    //Compensates for second order kinematic drift
    ChassisSpeeds.discretize(chassisSpeeds, 0.02); 
    var swerveModuleStates =
        Constants.m_kinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }


  /**
   * set desired state of each module, not currently used
   * @param swerveModuleStates
   */
  public void setModuleStates(SwerveModuleState[] swerveModuleStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }


  /**
   * Updates the field relative position of the robot.
   */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
  }


  /**
   * Get the pose of the robot
   * @return current odometry pose
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }


  /**
   * get the current speeds of all modules
   * @return current ChassisSpeeds
   */
  private ChassisSpeeds getChassisSpeeds() {
    return Constants.m_kinematics.toChassisSpeeds(getStates());
  }


  /**
   * get the state of each module
   * @return array of module states
   */
  private SwerveModuleState[] getStates() {
    return new SwerveModuleState[] {
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_backLeft.getState(),
      m_backRight.getState()
    };
  }


  /**
   * get the position of all modules
   * @return position of all modules
   */
  private SwerveModulePosition[] getPosition() {
    return new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_backLeft.getPosition(),
      m_backRight.getPosition()
    };
  }


  /**
   * reset the odometry object
   * @param pose new pose to set odometry to.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(m_gyro.getRotation2d(), getPosition(), pose);
  }


  /**
   * Reset the gyro heading
   */
  public void resetGyro() {
    m_gyro.zeroYaw();
  }


  /**
   * change all drive motors to brake mode
   */
  public void setDriveToBrake() {
    m_backLeft.changeDriveToBrake();
    m_backRight.changeDriveToBrake();
    m_frontLeft.changeDriveToBrake();
    m_frontRight.changeDriveToBrake();
  }


  /**
   * change all drive motors to coast mode
   */
  public void setDriveToCoast() {
    m_backLeft.changeDriveToCoast();
    m_backRight.changeDriveToCoast();
    m_frontLeft.changeDriveToCoast();
    m_frontRight.changeDriveToCoast();
  }


  /**
   * Standard Teleop driving
   */
  public void defaultMove() {
    limiter = 1.0;
      if (driveController.getRightTrigger() > 0.5) {
        limiter = 0.5;
      }
      // Get the x speed. We are inverting this because Xbox controllers return
      // negative values when we push forward.
      final var xSpeed = limiter * m_xspeedLimiter.calculate(
      Utils.squarePreserveSign(-MathUtil.applyDeadband(-driveController.getLeftStickY(), 0.1))
          * Constants.kMaxSpeed);

      // Get the y speed or sideways/strafe speed. We are inverting this because
      // we want a positive value when we pull to the left. Xbox controllers
      // return positive values when you pull to the right by default.
      final var ySpeed = limiter * m_yspeedLimiter.calculate(
          Utils.squarePreserveSign(MathUtil.applyDeadband(driveController.getLeftStickX(), 0.1))
              * Constants.kMaxSpeed);

      // Get the rate of angular rotation. We are inverting this because we want a
      // positive value when we pull to the left (remember, CCW is positive in
      // mathematics). Xbox controllers return positive values when you pull to
      // the right by default.
      final var rot = limiter * limiter * m_rotLimiter.calculate(
          Utils.cubePreserveSign(MathUtil.applyDeadband(-driveController.getRightStickX(), 0.1)
              * Constants.kMaxAngularSpeed));
    

      if ( DriverStation.isTeleop() ) {
        drive(xSpeed, ySpeed, rot, fieldRelative);
      }
  }


  /**
   * Drive the robot using sensor inputs for the rotation and driver input translation.
   * @param rotation rot/sec
   */
  public void lockonMove(double rotation) {
    limiter = 1.0;
      if (driveController.getRightTrigger() > 0.5) {
        limiter = 0.5;
      }
      // Get the x speed. We are inverting this because Xbox controllers return
      // negative values when we push forward.
      final var xSpeed = limiter * m_xspeedLimiter.calculate(
      Utils.squarePreserveSign(-MathUtil.applyDeadband(-driveController.getLeftStickY(), 0.1))
          * Constants.kMaxSpeed);

      // Get the y speed or sideways/strafe speed. We are inverting this because
      // we want a positive value when we pull to the left. Xbox controllers
      // return positive values when you pull to the right by default.
      final var ySpeed = limiter * m_yspeedLimiter.calculate(
          Utils.squarePreserveSign(MathUtil.applyDeadband(driveController.getLeftStickX(), 0.1))
              * Constants.kMaxSpeed);

      if ( DriverStation.isTeleop() ) {
        drive(xSpeed, ySpeed, -0.1 * rotation, fieldRelative);
      }
  }

  /**
   * Drive the robot using sensor inputs for the rotation and driver input translation.
   * @param rotation rot/sec
   */
  public void lockonMoveTowardsObject(double rotation, double speed) {
      if ( DriverStation.isTeleop() ) {
        SmartDashboard.putNumber("Rotation", -0.1 * rotation);
        drive(speed, 0, -0.05 * rotation, false);
      }
  }  

  
  /**
   * Get joystick values 
   * Set motor inputs
   */
  @Override 
    public void periodic(){
      // Update the pose
      updateOdometry();

      SmartDashboard.putBoolean("FieldRelative", fieldRelative);
      // SmartDashboard.putNumber("GyroHeading", m_gyro.getRotation2d().getDegrees());
    } 
     
}