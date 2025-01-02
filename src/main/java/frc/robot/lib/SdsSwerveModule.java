// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Utils.Constants;
import frc.robot.Utils.Values;

public class SdsSwerveModule {

  private SparkMax driveMotor;
  private SparkClosedLoopController driveMotorController;
  private SparkMaxConfig driveConfig;
  private SparkMax turningMotor;
  private SparkMaxConfig turningConfig;
  private SparkClosedLoopController turningPIDController;

  private int driveID;
  private int turnID;

  private int i;
  // Gains are for example purposes only - must be determined for your own robot!


  // Gains are for example purposes only - must be determined for your own robot!
  //private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
  //private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);
  
  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorCANId PWM output for the drive motor.
   * @param turningMotorCANId PWM output for the turning motor.
   * @param turningEncoderAnalogPort Analog input for the turning encoder channel A
   */
  public SdsSwerveModule(
      int driveMotorCANId,
      int turningMotorCANId) {

    driveMotor = new SparkMax(driveMotorCANId, MotorType.kBrushless);
    turningMotor = new SparkMax(turningMotorCANId, MotorType.kBrushless);
    driveConfig = new SparkMaxConfig();
    turningConfig = new SparkMaxConfig();
    driveConfig.idleMode(IdleMode.kCoast);
    turningConfig.idleMode(IdleMode.kBrake);
    driveConfig.smartCurrentLimit(40);
    driveConfig.secondaryCurrentLimit(80);

    driveID = driveMotorCANId;
    turnID = turningMotorCANId;

    driveConfig.closedLoop.pidf(Constants.swerveDriveMotorP, Constants.swerveDriveMotorI, Constants.swerveDriveMotorD, Constants.swerveDriveMotorFF);
    turningConfig.closedLoop.pid(Constants.swerveTurningP, Constants.swerveTurningI, Constants.swerveTurningD);

    turningPIDController = turningMotor.getClosedLoopController();
    turningPIDController.setFeedbackDevice(turningMotor.getAbsoluteEncoder(Type.kDutyCycle));

    
    //REVPhysicsSim.getInstance().addSparkMax(driveMotor, DCMotor.getNEO(1));
    //REVPhysicsSim.getInstance().addSparkMax(turningMotor, DCMotor.getVex775Pro(1));

    /*
     * native units of rpm to m/s
     */
    driveConfig.encoder.velocityConversionFactor((2.0 * Math.PI * Constants.kWheelRadius) / (Constants.kSecondsPerMinute * Constants.kGearRatio));
    // native units of revolutions to meters
    //driveMotor.getEncoder().setVelocityConversionFactor(Constants.kMaxSpeed/5700);
    driveConfig.encoder.positionConversionFactor((2.0 * Math.PI * Constants.kWheelRadius) / Constants.kGearRatio);
    driveMotorController = driveMotor.getClosedLoopController();
    

    turningPIDController.setPositionPIDWrappingEnabled(true);
    turningMotor.getAbsoluteEncoder(Type.kDutyCycle).setPositionConversionFactor(2*Math.PI);
    turningMotor.getAbsoluteEncoder(Type.kDutyCycle).setInverted(true);
    turningMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    turningMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);

    driveConfig.inverted(true);
  }


  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        driveMotor.getEncoder().getVelocity(), new Rotation2d(ConvertedTurningPosition()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveMotor.getEncoder().getPosition(), new Rotation2d(ConvertedTurningPosition()));
  }

  public void changeDriveToBrake() {
    driveConfig.idleMode(IdleMode.kBrake);
  }

  public void changeDriveToCoast() {
    driveConfig.idleMode(IdleMode.kCoast);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(ConvertedTurningPosition()));

    double convertedPosition = MathUtil.angleModulus(state.angle.getRadians()) + Math.PI;
      
    // if (i == 0) {
    //   System.out.println("Measured Angle   " + driveID + ":   " + ConvertedTurningPosition());
    //   System.out.println("Commanded Angle  " + driveID + ":   " + state.angle.getRadians());
    //   System.out.println("Commanded Speed " + driveID + ":   " + state.speedMetersPerSecond);
    //   System.out.println("Motor Speed     " + driveID + ": " + driveMotor.getEncoder().getVelocity());
    //   SmartDashboard.putNumber("Swerve Angle " + turnID, state.angle.getRadians());
    // }
    // i = (i + 1) % 100;

    // SmartDashboard.putNumber("Drive RPM" + driveID, driveMotor.getEncoder().getVelocity());
    // SmartDashboard.putNumber("Module Angle" + driveID,turningEncoder.getAbsolutePosition());
    // SmartDashboard.putNumber("Commanded Angle" + driveID, state.angle.getRadians());

    turningPIDController.setReference(convertedPosition, ControlType.kPosition);
    driveMotorController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
    //if (turnOutput > 0.5 ) {
    //  turningMotor.setVoltage(turnOutput);
    //} else {
    //  turningMotor.setVoltage(0);
    //}
  }

  private double ConvertedTurningPosition() {
    return turningMotor.getAbsoluteEncoder(ControlType.kDutyCycle).getPosition() - Math.PI;
  }

}