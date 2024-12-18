// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Utils.Constants;
import frc.robot.Utils.Values;

public class SdsSwerveModule {

  private double swerveTurningP;
  private double swerveTurningI;
  private double swerveTurningD;
  private double swerveDriveMotorP;
  private double swerveDriveMotorI;
  private double swerveDriveMotorD;
  private double swerveDriveMotorFF;

  private SparkMax driveMotor;
  private SparkClosedLoopController driveMotorController;
  private SparkMax turningMotor;
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
    turningMotor.setSmartCurrentLimit(40);
    turningMotor.setIdleMode(IdleMode.kBrake);
    driveMotor.setSecondaryCurrentLimit(80);
    driveMotor.setIdleMode(IdleMode.kCoast);
    driveID = driveMotorCANId;
    turnID = turningMotorCANId;

    turningPIDController = turningMotor.getClosedLoopController();
    turningPIDController.setFeedbackDevice(turningMotor.getAbsoluteEncoder(Type.kDutyCycle));

    swerveTurningP = Values.getInstance().getDoubleValue("swerveTurningP");
    swerveTurningI = Values.getInstance().getDoubleValue("swerveTurningI");
    swerveTurningD = Values.getInstance().getDoubleValue("swerveTurningD");
    swerveDriveMotorP = Values.getInstance().getDoubleValue("swerveDriveMotorP");
    swerveDriveMotorI = Values.getInstance().getDoubleValue("swerveDriveMotorI");
    swerveDriveMotorD = Values.getInstance().getDoubleValue("swerveDriveMotorD");
    swerveDriveMotorFF = Values.getInstance().getDoubleValue("swerveDriveMotorFF");
    

    //REVPhysicsSim.getInstance().addSparkMax(driveMotor, DCMotor.getNEO(1));
    //REVPhysicsSim.getInstance().addSparkMax(turningMotor, DCMotor.getVex775Pro(1));

    /*
     * native units of rpm to m/s
     */
    driveMotor.getEncoder().setVelocityConversionFactor((2.0 * Math.PI * Constants.kWheelRadius) / (Constants.kSecondsPerMinute * Constants.kGearRatio));
    // native units of revolutions to meters
    //driveMotor.getEncoder().setVelocityConversionFactor(Constants.kMaxSpeed/5700);
    driveMotor.getEncoder().setPositionConversionFactor((2.0 * Math.PI * Constants.kWheelRadius) / Constants.kGearRatio);
    driveMotorController = driveMotor.getClosedLoopController();
    driveMotorController.setP(swerveDriveMotorP);
    driveMotorController.setI(swerveDriveMotorI);
    driveMotorController.setD(swerveDriveMotorD);
    driveMotorController.setFF(swerveDriveMotorFF);

    turningPIDController.setPositionPIDWrappingEnabled(true);
    turningMotor.getAbsoluteEncoder(Type.kDutyCycle).setPositionConversionFactor(2*Math.PI);
    turningMotor.getAbsoluteEncoder(Type.kDutyCycle).setInverted(true);
    turningMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    turningMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);
    turningPIDController.setP(swerveTurningP);
    turningPIDController.setI(swerveTurningI);
    turningPIDController.setD(swerveTurningD);

    if (turnID == 4) {
      driveMotor.setInverted(true);
    } else {
      driveMotor.setInverted(false);
    }
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
    driveMotor.setIdleMode(IdleMode.kBrake);
  }

  public void changeDriveToCoast() {
    driveMotor.setIdleMode(IdleMode.kCoast);
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

    driveMotorController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
    turningPIDController.setReference(convertedPosition, ControlType.kPosition);
    //if (turnOutput > 0.5 ) {
    //  turningMotor.setVoltage(turnOutput);
    //} else {
    //  turningMotor.setVoltage(0);
    //}
  }

  private double ConvertedTurningPosition() {
    return turningMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition() - Math.PI;
  }

}