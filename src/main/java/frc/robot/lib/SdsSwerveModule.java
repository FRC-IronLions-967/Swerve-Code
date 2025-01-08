// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.Utils.Constants;

public class SdsSwerveModule {

  private SparkMax driveMotor;
  private SparkClosedLoopController driveMotorController;
  private SparkMaxConfig driveConfig;

  private SparkMax turningMotor;
  private SparkMaxConfig turningConfig;
  private SparkClosedLoopController turningMotorController;

  private SparkMaxSim driveMotorSim;
  private SparkMaxSim turningMotorSim;

  private int driveID;
  private int turnID;
  private Translation2d location;

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
      int turningMotorCANId,
      Translation2d moduleLocation) {

    driveMotor = new SparkMax(driveMotorCANId, MotorType.kBrushless);
    turningMotor = new SparkMax(turningMotorCANId, MotorType.kBrushless);
    driveConfig = new SparkMaxConfig();
    driveConfig
      .smartCurrentLimit(80)
      .idleMode(IdleMode.kCoast)
      .inverted(true);
    driveConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pidf(Constants.swerveDriveMotorP, Constants.swerveDriveMotorI, Constants.swerveDriveMotorD, Constants.swerveDriveMotorFF);
    driveConfig.encoder
      .velocityConversionFactor((2.0 * Math.PI * Constants.kWheelRadius) / (Constants.kSecondsPerMinute * Constants.kDriveGearRatio))
      .positionConversionFactor((2.0 * Math.PI * Constants.kWheelRadius) / Constants.kDriveGearRatio);
    driveMotor.configure(driveConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    turningConfig = new SparkMaxConfig();
    turningConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(40);
    turningConfig.closedLoop
      .pid(Constants.swerveTurningP, Constants.swerveTurningI, Constants.swerveTurningD)
      .positionWrappingInputRange(0, 2*Math.PI)
      .positionWrappingEnabled(true)
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    turningConfig.absoluteEncoder
      .inverted(true)
      .positionConversionFactor(2*Math.PI);
    turningMotor.configure(turningConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    
    driveMotorController = driveMotor.getClosedLoopController();
    turningMotorController = turningMotor.getClosedLoopController();

    driveMotorController.setReference(0.0, ControlType.kVelocity);
    turningMotorController.setReference(0.0, ControlType.kPosition);

    driveID = driveMotorCANId;
    turnID = turningMotorCANId;
    location = moduleLocation;

    // ----- Simulation support;
    driveMotorSim = new SparkMaxSim(driveMotor, DCMotor.getNEO(1));
    turningMotorSim = new SparkMaxSim(turningMotor, DCMotor.getNEO(1));
  }


  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        driveMotor.getEncoder().getVelocity(), Rotation2d.fromRadians(ConvertedTurningPosition()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveMotor.getEncoder().getPosition(), Rotation2d.fromRadians(ConvertedTurningPosition()));
  }

  public void changeDriveToBrake() {
    driveConfig.idleMode(IdleMode.kBrake);
    driveMotor.configure(driveConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }

  public void changeDriveToCoast() {
    driveConfig.idleMode(IdleMode.kCoast);
    driveMotor.configure(driveConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }


  /**
   * Used to convert encoder positions from (0, 2 PI) to (-PI, PI)
   * @return converted angle
   */
  private double ConvertedTurningPosition() {
    return turningMotor.getAbsoluteEncoder().getPosition() - Math.PI;
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    desiredState.optimize(new Rotation2d(ConvertedTurningPosition()));

    double convertedPosition = MathUtil.angleModulus(desiredState.angle.getRadians()) + Math.PI;

    turningMotorController.setReference(convertedPosition, ControlType.kPosition);
    driveMotorController.setReference(desiredState.speedMetersPerSecond, ControlType.kVelocity);
  }
  
  /** Module heading reported by steering encoder. */
  public Rotation2d getAbsoluteHeading() {
    return Rotation2d.fromRadians(ConvertedTurningPosition());
  }


  public Translation2d getModuleLocation() {
    return location;
  }

  public double getDriveVoltage(){
    return driveMotor.getAppliedOutput() * 12.0;
  }

  
  public double getSteerVoltage(){
    return turningMotor.getAppliedOutput() * 12.0;
  }

  public void simulationUpdate(
            double drivePos,
            double driveRate,
            double driveCurrent,
            double steerPos,
            double steerRate,
            double steerCurrent) {
    double steerRateRot = Units.radiansPerSecondToRotationsPerMinute(steerRate);
    driveMotorSim.setPosition(drivePos);
    driveMotorSim.iterate(driveRate, 12.0, Robot.kDefaultPeriod);
    turningMotorSim.iterate(steerRateRot / Constants.kSteerGearRatio, 12.0, Robot.kDefaultPeriod);
    turningMotorSim.setPosition(MathUtil.angleModulus(steerPos) + Math.PI);
    turningMotorSim.getAbsoluteEncoderSim().setPosition(MathUtil.angleModulus(steerPos) + Math.PI);
      
  }
}