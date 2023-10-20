// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;

public final class Autos {
  private static final PIDConstants kTranslationalPID = new PIDConstants(6.0, 0.0, 0.2);
  private static final PIDConstants kRotationalPID = new PIDConstants(5.0, 0.0, 0.2);
  private static final double kMaxModuleSpeed = Units.feetToMeters(16.0);

  public static void configure(Drive drive) {
    HolonomicPathFollowerConfig config = new HolonomicPathFollowerConfig(kTranslationalPID, kRotationalPID,
        kMaxModuleSpeed, Units.inchesToMeters(15), new ReplanningConfig(true, true));

    AutoBuilder.configureHolonomic(
        drive::getPose,
        drive::resetPose,
        drive::getChassisSpeeds,
        drive::drive,
        config,
        drive);

    NamedCommands.registerCommand("a", Commands.print("Command A"));
    NamedCommands.registerCommand("b", Commands.print("Command B"));
    NamedCommands.registerCommand("c", Commands.print("Command C"));
    NamedCommands.registerCommand("hello", Commands.print("Hello"));
    NamedCommands.registerCommand("brake", drive.brakeCommand());
  }

  /** Example static factory for an autonomous command. */
  public static Command getAutoByName(String name) {
    try {
      return AutoBuilder.buildAuto(name);
    } catch (Exception e) {
      DriverStation.reportError("An error occurred while loading path planner auto", e.getStackTrace());
      return Commands.none();
    }
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
