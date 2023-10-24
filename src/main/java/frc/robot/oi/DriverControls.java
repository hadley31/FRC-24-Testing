package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface DriverControls {
  public double driveForward();

  public double driveLeft();

  public double driveRotate();

  public double driverRightX();

  public double driverRightY();

  public Trigger robotRelativeDrive();

  public Trigger seedFieldRelative();
}
