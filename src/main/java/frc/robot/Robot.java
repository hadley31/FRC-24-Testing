// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.Map;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommands;
import frc.robot.oi.DriverControls;
import frc.robot.oi.SingleUserXboxControls;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.AprilTagCamera;

public class Robot extends TimedRobot {
  private RobotState m_state = new RobotState();
  private Drive m_drive;
  private Arm m_arm;
  private Map<String, AprilTagCamera> m_cameras;

  private DriverControls m_driverControls;

  private AprilTagFieldLayout m_aprilTagLayout;

  private VisionSystemSim m_visionSim;

  private Command m_autonomousCommand;

  private final SendableChooser<Supplier<Command>> m_autoChooser = new SendableChooser<Supplier<Command>>();

  @Override
  public void robotInit() {
    DataLogManager.start("logs");
    DataLogManager.logNetworkTables(true);
    DriverStation.startDataLog(DataLogManager.getLog(), true);

    configureSubsystems();

    configureBindings();

    configureAllianceSettings(DriverStation.getAlliance().orElse(null));

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

  @Override
  public void simulationInit() {
    m_visionSim = new VisionSystemSim("Vision Sim");
    m_visionSim.addAprilTags(m_aprilTagLayout);
    m_cameras.values().forEach(c -> m_visionSim.addCamera(new PhotonCameraSim(c.getCamera()), c.getRobotToCamera()));
  }

  @Override
  public void simulationPeriodic() {
    m_visionSim.update(m_drive.getPose3d());
  }

  private void configureSubsystems() {
    m_aprilTagLayout = getAprilTagLayout();

    if (Robot.isReal()) {
      configureRealSubsystems();
    } else {
      configureSimSubsystems();
    }
  }

  private void configureRealSubsystems() {
    m_drive = new Drive(m_state, CTRESwerveConfig.getConfiguredSwerveDrivetrain());
  }

  private void configureSimSubsystems() {
    m_drive = new Drive(m_state, CTRESwerveConfig.getConfiguredSwerveDrivetrain());
    m_arm = new Arm(20, 21);

    var camera1 = new PhotonCamera(VisionConstants.camera1Name);
    // var camera2 = new PhotonCamera(VisionConstants.camera2Name);

    var cameraSub1 = new AprilTagCamera(camera1, VisionConstants.robotToCamera1, m_aprilTagLayout, m_state);
    // var cameraSub2 = new AprilTagCamera(camera2, VisionConstants.robotToCamera2, m_aprilTagLayout, m_state);

    // m_cameras = Map.of(
    //     camera1.getName(), cameraSub1,
    //     camera2.getName(), cameraSub2);

    m_cameras = Map.of(camera1.getName(), cameraSub1);

    m_cameras.values().forEach(c -> c.setPoseListener(poseResult -> {
      m_drive.getSwerve().addVisionMeasurement(
          poseResult.estimatedPose.toPose2d(),
          poseResult.timestampSeconds,
          c.getStdDeviations());
    }));
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

  private void configureAllianceSettings(Alliance alliance) {
    var origin = alliance == Alliance.Red
        ? OriginPosition.kRedAllianceWallRightSide
        : OriginPosition.kBlueAllianceWallRightSide;

    m_aprilTagLayout.setOrigin(origin);
  }

  private void configureAutos() {
    Autos.configure(m_drive);

    m_autoChooser.setDefaultOption("Do Nothing", () -> Commands.none());

    PathPlannerUtil.getExistingPaths()
        .forEach(pathName -> m_autoChooser.addOption(pathName,
            () -> Autos.getAutoByName(pathName).andThen(m_drive.brakeCommand())));
    SmartDashboard.putData("Auto Chooser", m_autoChooser);
  }

  private AprilTagFieldLayout getAprilTagLayout() {
    try {
      return AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
    } catch (IOException e) {
      DriverStation.reportError("Failed to load apriltag field layout", e.getStackTrace());
      throw new RuntimeException("Failed to load apriltag field layout");
    }
  }
}
