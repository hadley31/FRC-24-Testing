// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.commands.drive.DriveCommandConfig;
import frc.lib.pathplanner.PathPlannerUtil;
import frc.lib.swerve.CTRESwerveConfig;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommands;
import frc.robot.oi.DriverControls;
import frc.robot.oi.SingleUserXboxControls;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.drive.Drive;

public class Robot extends TimedRobot {
  private Drive m_drive;
  private Arm m_arm;

  private DriverControls m_driverControls;

  private Command m_autonomousCommand;

  private final SendableChooser<Supplier<Command>> m_autoChooser = new SendableChooser<Supplier<Command>>();

  @Override
  public void robotInit() {
    DataLogManager.start("logs");
    DataLogManager.logNetworkTables(true);

    configureSubsystems();

    configureBindings();

    configureAutos();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_autoChooser.getSelected().get();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  private void configureSubsystems() {
    if (Robot.isReal()) {
      configureRealSubsystems();
    } else {
      configureSimSubsystems();
    }
  }

  private void configureRealSubsystems() {
    m_drive = new Drive(CTRESwerveConfig.getConfiguredSwerveDrivetrain());
  }

  private void configureSimSubsystems() {
    m_drive = new Drive(CTRESwerveConfig.getConfiguredSwerveDrivetrain());
    m_arm = new Arm(20, 21);
  }

  private void configureBindings() {
    var singleUserControls = new SingleUserXboxControls(0);
    m_driverControls = singleUserControls;

    DriveCommandConfig config = new DriveCommandConfig(
        DriveConstants.kDriveKinematics,
        m_drive::getPose,
        m_drive::getModuleStates,
        () -> 0.0,
        m_drive::drive,
        m_drive);

    var fieldRelativeCommand = DriveCommands.fieldRelative(config, m_driverControls);
    var robotRelativeCommand = DriveCommands.robotRelative(config, m_driverControls);

    m_drive.setDefaultCommand(fieldRelativeCommand);

    m_driverControls.robotRelativeDrive().whileTrue(robotRelativeCommand);

    m_driverControls.seedFieldRelative().onTrue(m_drive.seedFieldRelativeCommand());

    m_arm.setTargetAngleCommand(Rotation2d.fromDegrees(20));
  }

  private void configureAutos() {
    Autos.configure(m_drive);

    m_autoChooser.setDefaultOption("Do Nothing", () -> Commands.none());

    PathPlannerUtil.getExistingPaths()
        .forEach(pathName -> m_autoChooser.addOption(pathName,
            () -> Autos.getAutoByName(pathName).andThen(m_drive.brakeCommand())));
    SmartDashboard.putData("Auto Chooser", m_autoChooser);
  }
}
