package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.lib.commands.drive.DriveCommand;
import frc.lib.commands.drive.DriveCommandConfig;
import frc.robot.oi.DriverControls;

public class DriveCommands {
  public static DriveCommand fieldRelative(DriveCommandConfig config, DriverControls controls) {
    return DriveCommand.builder(config)
        .withLinearSpeedSuppliers(
            controls::driveForward,
            controls::driveLeft)
        .withAngularSpeedSupplier(controls::driveRotate)
        .withSpeedConstraints(Units.feetToMeters(9), Units.degreesToRadians(360))
        .usePercentSpeeds(true)
        .useFieldRelative(true)
        .build();
  }

  public static DriveCommand faceAngle(DriveCommandConfig config, DriverControls controls,
      Rotation2d angle) {
    Constraints constraints = new Constraints(Units.degreesToRadians(1280), Units.degreesToRadians(7200));
    ProfiledPIDController controller = new ProfiledPIDController(10, 0, 0.2, constraints, 0.02);

    Supplier<Rotation2d> angleSupplier = () -> {
      Rotation2d rot = new Rotation2d(MathUtil.applyDeadband(controls.driverRightY(), 0.5),
          MathUtil.applyDeadband(-controls.driverRightX(), 0.5));

      return Rotation2d.fromDegrees(Math.round(rot.getDegrees() / 90.0) * 90);
    };

    return DriveCommand.builder(config)
        .withLinearSpeedSuppliers(
            controls::driveForward,
            controls::driveLeft)
        .withTargetAngleSupplier(angleSupplier, controller)
        .withSpeedConstraints(Units.feetToMeters(9), Units.degreesToRadians(1280))
        .usePercentSpeeds(true)
        .useFieldRelative(true)
        .build();
  }
}
