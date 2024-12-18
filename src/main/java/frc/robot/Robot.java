// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.SubsystemsInstance;
import frc.robot.commands.*;
import frc.robot.lib.LEDController;

public class Robot extends TimedRobot {
  private SubsystemsInstance subsystemsInst;
  private Command m_autonomousCommand;
  private SendableChooser<Command> autoChooser;
  private LEDController ledController;
  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    subsystemsInst = SubsystemsInstance.getInstance();
    subsystemsInst.drivetrain.setupPathPlanner();
    CommandScheduler.getInstance().setDefaultCommand(subsystemsInst.drivetrain, new DefaultMoveCommand());


    autoChooser = AutoBuilder.buildAutoChooser("Center Simple_Auto");
    SmartDashboard.putData("Auto Chooser", autoChooser);

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
    SubsystemsInstance.getInstance().drivetrain.setDriveToBrake();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    ledController.enabledPeriodic();
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    IO.getInstance().teleopInit();
    SubsystemsInstance.getInstance().drivetrain.setDriveToCoast();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    ledController.enabledPeriodic();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    ledController.disabledPeriodic();
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
