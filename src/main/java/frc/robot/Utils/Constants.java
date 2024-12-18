package frc.robot.Utils;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DoNothingCommand;

public final class Constants {

    public static final double kDriveRadius = Math.sqrt(0.308 * 0.308 + 0.308 * 0.308); //radius from center of drive to one module
    public static final double kMaxSpeed = 4.42; // 4.42 meters per second / 14.5 ft per second
    public static final double kMaxAcceleration = 10.0; // 6.0 meters per second per second
    public static final double kMaxAngularSpeed = kMaxSpeed / kDriveRadius; // Maximum angular velocity 
    public static final double kMaxAngularAcceleration = kMaxAcceleration / kDriveRadius; // Maximum angular acceleration
    public static final double kWheelRadius = 0.0489;
    //the number above is acurate
    public static final double kGearRatio = 6.75;
    //needs tunings
    public static final double kSecondsPerMinute = 60.0;

    public static final double objectDetectionCameraYawOffset = 0.0;
    public static final double kMaxNeoSpeed = 5676.0;

    public static final double swerveTurningP = 1.5;
    public static final double swerveTurningI = 0.0;
    public static final double swerveTurningD = 0.5;
    public static final double swerveDriveMotorP = 0.08;
    public static final double swerveDriveMotorI = 0.0;
    public static final double swerveDriveMotorD = 0.025;
    public static final double swerveDriveMotorFF = 0.28;
  
    // private static final double kModuleMaxAngularVelocity = kMaxAngularSpeed;
    // private static final double kModuleMaxAngularAcceleration =
    //     2 * Math.PI; // radians per second squared
    public static final Translation2d m_frontLeftLocation = new Translation2d(0.308, 0.308);
    public static final Translation2d m_frontRightLocation = new Translation2d(0.308, -0.308);
    public static final Translation2d m_backLeftLocation = new Translation2d(-0.308, 0.308);
    public static final Translation2d m_backRightLocation = new Translation2d(-0.308, -0.308);
      //real numbers xare put in above
    public static final SwerveDriveKinematics m_kinematics =
    new SwerveDriveKinematics(
       m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
    new TrapezoidProfile.Constraints(
        kMaxAngularSpeed, kMaxAngularAcceleration);

    public static final Map<String, Command> autoEventMap = new HashMap<>();
    static {
      autoEventMap.put("autoEvent1", new DoNothingCommand());
      autoEventMap.put("autoEvent2", new DoNothingCommand());
      autoEventMap.put("autoEvent3", new DoNothingCommand());
    }

    public static final Map<Integer, Double> kSwerveOffsets = new HashMap<>();
    static {
      kSwerveOffsets.put(0, 2.017568133691152);
      kSwerveOffsets.put(1, 1.6252273289298291);
      kSwerveOffsets.put(2, -2.7157190558997755);
      kSwerveOffsets.put(3, 0.23217007031624007);
    }

    public static final Map<Double, Double> swerveRotMap = new HashMap<>();
    static {
      double y;
      y = 0.00;
      for (double x = 0.00; x < 1.01; x = x + 0.01) {
        if (x < -0.90) {
          y = y - 0.03;//-1.00
          swerveRotMap.put(x, y);
        }
        if (x < -0.80 && x > -0.90) {
          y = y - 0.015;//-0.70
          swerveRotMap.put(x, y);
        }
        if (x < -0.70 && x > -0.80) {
          y = y - 0.013;//-0.55
          swerveRotMap.put(x, y);
        }
        if (x < -0.60 && x > -0.70) {
          y = y - 0.011;//-0.42
          swerveRotMap.put(x, y);
        }
        if (x < -0.50 && x > -0.60) {
          y = y - 0.008;//-0.31
          swerveRotMap.put(x, y);
        }
        if (x < -0.40 && x > -0.50) {
          y = y - 0.007;//-0.23
          swerveRotMap.put(x, y);
        }
        if (x < -0.30 && x > -0.40) {
          y = y - 0.006;//-0.16
          swerveRotMap.put(x, y);
        }
        if (x < -0.20 && x > -0.30) {
          y = y - 0.005;//-0.10
          swerveRotMap.put(x, y);
        }
        if (x < -0.10 && x > -0.20) {
          y = y - 0.005;//-0.05
          swerveRotMap.put(x, y);
        }
        if (x < 0.10 && x > -0.10) {
          y = 0;//0.00
          swerveRotMap.put(x, y);
        }
        if (x >= 0.1 && x < 0.2) {
          y = y + 0.005;//0.05
          swerveRotMap.put(x, y);
        }
        if (x >= 0.2 && x < 0.3) {
          y = y + 0.005;//0.10
          swerveRotMap.put(x, y);
        }
        if (x >= 0.3 && x < 0.4) {
          y = y + 0.006;//0.16
          swerveRotMap.put(x, y);        
        }
        if (x >= 0.4 && x < 0.5) {
          y = y + 0.007;//0.23
          swerveRotMap.put(x, y);        
        }
        if (x >= 0.5 && x < 0.6) {
          y = y + 0.008;//0.31
          swerveRotMap.put(x, y);        
        }
        if (x >= 0.6 && x < 0.7) {
          y = y + 0.011;//0.42
          swerveRotMap.put(x, y);        
        }
        if (x >= 0.7 && x < 0.8) {
          y = y + 0.013;//0.55
          swerveRotMap.put(x, y);        
        }
        if (x >= 0.8 && x < 0.9) {
          y = y + 0.015;//0.70
          swerveRotMap.put(x, y);        
        }
        if (x >= 0.9) {
          y = y + 0.03;//1.00
          swerveRotMap.put(x, y);          
        }
      }
    }

}
