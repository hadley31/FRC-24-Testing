package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
  public static final double loopPeriodSecs = 0.02;
  public static final double kTwoPi = 2 * Math.PI;

  public static final class DriveConstants {
    public static final double kWheelDiameter = Units.inchesToMeters(3.87);

    public static final double kWheelBase = Units.inchesToMeters(23);
    public static final double kTrackWidth = Units.inchesToMeters(23);

    public static final Translation2d[] kSwerveModuleTranslations = {
        new Translation2d(kTrackWidth / 2.0, kWheelBase / 2.0), //values for front left (+, +)
        new Translation2d(kTrackWidth / 2.0, -kWheelBase / 2.0), //values for front right (+, -)
        new Translation2d(-kTrackWidth / 2.0, kWheelBase / 2.0), //values for back left (-, +)
        new Translation2d(-kTrackWidth / 2.0, -kWheelBase / 2.0) //values for back right (-, -)
    };

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(kSwerveModuleTranslations);

    // Gear Ratios
    public static final double kDriveGearRatio = 6.75;
    public static final double kTurnGearRatio = 150.0 / 7.0;

    // Encoder Conversion Factors
    public static final double kDrivePositionConversionFactor = Math.PI * kWheelDiameter / kDriveGearRatio;
    public static final double kDriveVelocityConversionFactor = kDrivePositionConversionFactor / 60;
    public static final double kTurnPositionConversionFactor = 2 * Math.PI / kTurnGearRatio;
    public static final double kTurnVelocityConversionFactor = 2 * kTurnPositionConversionFactor / 60;

    // PID Values
    public static final double kDriveP = 0.1;
    public static final double kDriveI = 0.0;
    public static final double kDriveD = 0.001;
    public static final double kDriveS = 0;
    public static final double kDriveV = 0;
    public static final double kDriveG = 0;
    public static final double kDriveA = 0;

    public static final double kTurnP = 0.1;
    public static final double kTurnI = 0.0;
    public static final double kTurnD = 0.005;
    public static final double kTurnFF = 0.005;
    public static final double kTurnS = 0;
    public static final double kTurnV = 0;
    public static final double kTurnG = 0;
    public static final double kTurnA = 0;

    // Max Speeds
    public static final double kMaxSpeedMetersPerSecond = 4.0;
    public static final double kMaxSpeedRadiansPerSecond = Units.degreesToRadians(360);
  }

  public static final class FieldConstants {
    public static final double kFieldLengthMeters = Units.feetToMeters(54.27083);
    public static final double kFieldWidthMeters = Units.feetToMeters(26.2916);
    public static final Pose2d kOppositeField = new Pose2d(kFieldLengthMeters, kFieldWidthMeters,
        Rotation2d.fromDegrees(180));
  }
}
