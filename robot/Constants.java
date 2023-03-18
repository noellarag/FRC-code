package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static class DrivetrainConstants {
    public static final int kDriverControllerPort = 0;
    public static final int leftCANID = 1;
    public static final int RightCANID = 2;

    public static final double KsVolts = 0.11483;
    public static final double ksVoltSecondsPerMeter = 2.2004;
    public static final double ksVoltSecondsSquaredperMeter = 0.51859;
    public static final double kPDriveVel = 3.0711;

    public static final double kTrackwidtMeter = 0.54; //horizontal distance between two wheels
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidtMeter);

    public static final double kMaxSpeedMetersPerSecond = 0.8;
    public static final double kMaxAccelerationPerSecondSquared = 0.8;
  
    public static final double kRamseteB = 2;
    public static final double kRamsetZeta = 0.7;

    public static final double kGearRatio = 10.71;
    public static final double kWheelRadiusIn = 0.0762;

    public static final double kLinearDistanceConversionFactor = (Units.inchesToMeters (1 / (kGearRatio * 2 * Math.PI * kWheelRadiusIn) * 11.71));
  }
}