// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.SubsystemsInst;
import frc.robot.commands.*;
import frc.robot.lib.LEDController;

public class Robot extends TimedRobot {
  private SubsystemsInst subsystemsInst;
  private Command m_autonomousCommand;
  private SendableChooser<Command> autoChooser;
  private LEDController ledController;
  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    subsystemsInst = SubsystemsInst.getInst();
    subsystemsInst.drivetrain.setupPathPlanner();
    CommandScheduler.getInstance().setDefaultCommand(subsystemsInst.drivetrain, new DefaultMoveCommand());


    // autoChooser = AutoBuilder.buildAutoChooser("Center Simple_Auto");
    // SmartDashboard.putData("Auto Chooser", autoChooser);

    ledController = new LEDController(0, 36);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    ledController.heartbeat();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = autoChooser.getSelected();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
     m_autonomousCommand.schedule();
    }
    SubsystemsInst.getInst().drivetrain.setDriveToBrake();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    //ledController.enabledPeriodic();
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    IO.getInstance().teleopInit();
    SubsystemsInst.getInst().drivetrain.setDriveToCoast();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //ledController.enabledPeriodic();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    //ledController.disabledPeriodic();
  }

  @Override
  public void simulationInit() {
    // Example Only - startPose should be derived from some assumption
    // of where your robot was placed on the field.
    // The first pose in an autonomous path is often a good choice.
    var startPose = new Pose2d(1, 1, new Rotation2d());
    SubsystemsInst.getInst().drivetrain.resetOdometry(startPose);
    SubsystemsInst.getInst().vision.resetSimPose(startPose);
  }

  @Override
  public void simulationPeriodic() {
      // Update drivetrain simulation
      SubsystemsInst.getInst().drivetrain.simulationPeriodic();

      // Update camera simulation
      SubsystemsInst.getInst().vision.simulationPeriodic(SubsystemsInst.getInst().drivetrain.getSimPose());

      var debugField = SubsystemsInst.getInst().vision.getSimDebugField();
      debugField.getObject("EstimatedRobot").setPose(SubsystemsInst.getInst().drivetrain.getPose());
      debugField.getObject("EstimatedRobotModules").setPoses(SubsystemsInst.getInst().drivetrain.getModulePoses());


      // // Calculate battery voltage sag due to current draw
      // var batteryVoltage =
      //         BatterySim.calculateDefaultBatteryLoadedVoltage(drivetrain.getCurrentDraw());

      // // Using max(0.1, voltage) here isn't a *physically correct* solution,
      // // but it avoids problems with battery voltage measuring 0.
      // RoboRioSim.setVInVoltage(Math.max(0.1, batteryVoltage));
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
