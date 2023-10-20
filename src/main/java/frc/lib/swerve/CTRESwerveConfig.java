package frc.lib.swerve;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SwerveModuleSteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;

public class CTRESwerveConfig {
  private static final Slot0Configs kDriveSlotConfigs = new Slot0Configs() {
    {
      kP = 10.5;
      kI = 0.1;
      kD = 0.1;
      kV = 0.6;
      kS = 0.1;
      kG = 0.0;
      kA = 0.0;
    }
  };
  private static final Slot0Configs kTurnSlotConfigs = new Slot0Configs() {
    {
      kP = 15.0;
      kI = 0.2;
      kD = 0.2;
      kV = 2.0;
      kS = 0.3;
      kG = 0.0;
      kA = 0.0;
    }
  };

  private static final SwerveModuleConstantsFactory kConstantCreator = new SwerveModuleConstantsFactory()
      .withDriveMotorGearRatio(DriveConstants.kDriveGearRatio)
      .withSteerMotorGearRatio(DriveConstants.kTurnGearRatio)
      .withWheelRadius(Units.metersToInches(DriveConstants.kWheelDiameter / 2))
      .withSlipCurrent(800)
      .withSteerMotorGains(kTurnSlotConfigs)
      .withDriveMotorGains(kDriveSlotConfigs)
      .withSpeedAt12VoltsMps(Units.feetToMeters(16)) // Theoretical free speed is 10 meters per second at 12v applied output
      .withSteerInertia(0.1)
      .withDriveInertia(0.1)
      .withFeedbackSource(SwerveModuleSteerFeedbackType.RemoteCANcoder)
      .withCouplingGearRatio(0.0) // Every 1 rotation of the azimuth results in couple ratio drive turns
      .withSteerMotorInverted(false);

  public static Swerve getConfiguredSwerveDrivetrain() {
    var drivetrain = new SwerveDrivetrainConstants()
        .withPigeon2Id(1)
        .withCANbusName("canivore")
        .withSupportsPro(false);

    var frontLeftModule = generateModuleConstants(2, 3, 4, DriveConstants.kSwerveModuleTranslations[0]);
    var frontRightModule = generateModuleConstants(5, 6, 7, DriveConstants.kSwerveModuleTranslations[1]);
    var backLeftModule = generateModuleConstants(8, 9, 10, DriveConstants.kSwerveModuleTranslations[2]);
    var backRightModule = generateModuleConstants(11, 12, 13, DriveConstants.kSwerveModuleTranslations[3]);

    return new Swerve(
        drivetrain,
        frontLeftModule,
        frontRightModule,
        backLeftModule,
        backRightModule);
  }

  private static SwerveModuleConstants generateModuleConstants(int turnId, int driveId, int cancoderId,
      Translation2d modulePosition) {
    double x = modulePosition.getX();
    double y = modulePosition.getY();
    double absoluteOffset = 0.0;

    return kConstantCreator.createModuleConstants(turnId, driveId, cancoderId, absoluteOffset, x, y, false);
  }
}
