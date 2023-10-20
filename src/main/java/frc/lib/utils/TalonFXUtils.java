package frc.lib.utils;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;

public final class TalonFXUtils {

  public static final double kTicksPerRevolution = 2048;
  private static final double kDegrees = 360.0;
  private static final double kRadians = 2 * Math.PI;
  private static final double kIdentity = 1.0;
  private static final double kDeciSecondsToSeconds = 10;
  private static final double kSecondsToDeciSeconds = 1 / kDeciSecondsToSeconds;
  private static final double kDeciSecondsToMinutes = kDeciSecondsToSeconds * 60;
  private static final double kMinutesToDeciSeconds = 1 / kDeciSecondsToMinutes;

  /**
   * Converts ticks per 100ms to radians per second.
   * @param ticksPer100ms ticks per 100ms
   * @param scalingFactor scaling factor to account for gear ratios, etc.
   * @return radians per second
   */
  public static double ticksPer100msToRadsPerSecond(double ticksPer100ms, double scalingFactor) {
    return ticksToRadians(ticksPer100ms, scalingFactor) * kDeciSecondsToSeconds;
  }

  /**
   * Converts ticks per 100ms to radians per second.
   * @param radsPerSecond ticks per 100ms
   * @param scalingFactor scaling factor to account for gear ratios, etc.
   * @return radians per second
   */
  public static double radsPerSecondTicksPer100ms(double radsPerSecond, double scalingFactor) {
    return radiansToTicks(radsPerSecond, scalingFactor) * kSecondsToDeciSeconds;
  }

  /**
   * Converts ticks per 100ms to revolutions per minute.
   * @param ticks ticks per 100ms
   * @return RPM
   */
  public static double ticksPer100msToRPM(double ticks) {
    return scaleFromTicks(ticks, kDeciSecondsToMinutes, kIdentity);
  }

  /**
   * Converts revolutions per minute ticks per 100ms.
   * @param rpm revolutions per minute
   * @return ticks per 100ms
   */
  public static double RPMToTicksPer100ms(double rpm) {
    return scaleToTicks(rpm, kMinutesToDeciSeconds, kIdentity);
  }

  public static double metersPerSecondToTicksPer100ms(double metersPerSecond, double scalingFactor) {
    return metersToTicks(metersPerSecond, scalingFactor) * kSecondsToDeciSeconds;
  }

  public static double ticksPer100msToMetersPerSecond(double ticksPer100ms, double scalingFactor) {
    return ticksToMeters(ticksPer100ms, scalingFactor) * kDeciSecondsToSeconds;
  }

  public static double ticksPer100msToMetersPerSecond(double ticksPer100ms, double gearRatio, double wheelDiameter) {
    return ticksToMeters(ticksPer100ms, gearRatio, wheelDiameter) * kDeciSecondsToSeconds;
  }

  public static double metersToTicks(double meters, double gearRatio, double wheelDiameter) {
    double circumference = Math.PI * wheelDiameter;
    return scaleToTicks(meters, circumference, gearRatio);
  }

  public static double ticksToMeters(double ticks, double gearRatio, double wheelDiameter) {
    double circumference = Math.PI * wheelDiameter;
    return scaleFromTicks(ticks, circumference, gearRatio);
  }

  /**
  * Use this if you already have the scalingFactor pre-calculated
  * @param meters Distance
  * @param scalingFactor PI * wheelDiameter / gearRatio
  * @return
  */
  public static double metersToTicks(double meters, double scalingFactor) {
    return scaleToTicks(meters, kIdentity, scalingFactor);
  }

  /**
   * Use this if you already have the scalingFactor pre-calculated
   * @param ticks Motor encoder tick reading
   * @param scalingFactor PI * wheelDiameter / gearRatio
   * @return
   */
  public static double ticksToMeters(double ticks, double scalingFactor) {
    return scaleFromTicks(ticks, kIdentity, scalingFactor);
  }

  /**
   * Converts from radians to native encoder ticks
   * @param radians of the motor rotor
   * @return native encoder tick units (2048 per rotation)
   */
  public static double radiansToTicks(double radians) {
    return radiansToTicks(radians, kIdentity);
  }

  /**
   * @param ticks Motor encoder tick reading
   * @return radians of the motor rotor
   */
  public static double ticksToRadians(double ticks) {
    return ticksToRadians(ticks, kIdentity);
  }

  /**
   * Converts from degrees to native encoder ticks
   * @param degrees of the motor rotor
   * @return native encoder tick units (2048 per rotation)
   */
  public static double degreesToTicks(double degrees) {
    return degreesToTicks(degrees, kIdentity);
  }

  public static double ticksToDegrees(double ticks) {
    return ticksToDegrees(ticks, kIdentity);
  }

  public static double rotation2dToTicks(Rotation2d rotation2d, double scalingFactor) {
    return radiansToTicks(rotation2d.getRadians(), scalingFactor);
  }

  public static Rotation2d ticksToRotation2d(double ticks, double scalingFactor) {
    return Rotation2d.fromRadians(ticksToRadians(ticks, scalingFactor));
  }

  public static double radiansToTicks(double radians, double scalingFactor) {
    return scaleToTicks(radians, kRadians, scalingFactor);
  }

  public static double ticksToRadians(double ticks, double scalingFactor) {
    return scaleFromTicks(ticks, kRadians, scalingFactor);
  }

  public static double degreesToTicks(double degrees, double scalingFactor) {
    return scaleToTicks(degrees, kDegrees, scalingFactor);
  }

  public static double ticksToDegrees(double ticks, double scalingFactor) {
    return scaleFromTicks(ticks, kDegrees, scalingFactor);
  }

  /**
   * (ticks / kTicksPerRevolution) * staticFactor / scalingFactor
   * @param ticks
   * @param staticFactor
   * @param scalingFactor
   * @return
   */
  private static double scaleFromTicks(double ticks, double staticFactor, double scalingFactor) {
    return (ticks / kTicksPerRevolution) * staticFactor / scalingFactor;
  }

  /**
   * (units / staticFactor) * kTicksPerRevolution * scalingFactor
   * @param units
   * @param staticFactor
   * @param scalingFactor
   * @return
   */
  private static double scaleToTicks(double units, double staticFactor, double scalingFactor) {
    return (units / staticFactor) * kTicksPerRevolution * scalingFactor;
  }

  public static void setPIDF(TalonFX motor, double kP, double kI, double kD, double kS, double kV, double kG, double kA) {
    Slot0Configs config = new Slot0Configs();

    config.kP = kP;
    config.kI = kI;
    config.kD = kD;

    config.kS = kS;
    config.kV = kV;
    config.kG = kG;
    config.kA = kA;

    motor.getConfigurator().apply(config);
  }
}
