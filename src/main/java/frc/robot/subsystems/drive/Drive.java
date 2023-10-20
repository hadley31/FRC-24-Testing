package frc.robot.subsystems.drive;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.RobotCentric;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SwerveDriveBrake;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.advantagekit.DataLogUtil;
import frc.lib.pathplanner.PathPlannerUtil;
import frc.lib.swerve.Swerve;
import frc.lib.utils.FieldUtil;
import frc.robot.Constants.DriveConstants;

public class Drive extends SubsystemBase {

  private Swerve m_swerve;

  DoubleArrayPublisher m_currentPosePub = NetworkTableInstance.getDefault().getDoubleArrayTopic("/Drive/CurrentPose")
      .publish();
  DoubleArrayPublisher m_desiredPathPosePub = NetworkTableInstance.getDefault().getDoubleArrayTopic("/Drive/TargetPose")
      .publish();
  DoubleArrayPublisher m_desiredSpeedsPub = NetworkTableInstance.getDefault()
      .getDoubleArrayTopic("/Drive/DesiredSpeeds").publish();
  DoubleArrayPublisher m_currentStatesPub = NetworkTableInstance.getDefault()
      .getDoubleArrayTopic("/Drive/CurrentStates").publish();
  DoubleArrayPublisher m_desiredStatesPub = NetworkTableInstance.getDefault()
      .getDoubleArrayTopic("/Drive/DesiredStates").publish();
  DoubleArrayPublisher m_currentAbsoluteStatesPub = NetworkTableInstance.getDefault()
      .getDoubleArrayTopic("/Drive/CurrentAbsoluteStates").publish();

  private final SwerveRequest.RobotCentric m_robotCentricRequest = new RobotCentric();
  private final SwerveRequest.FieldCentric m_fieldCentricRequest = new FieldCentric();
  private final SwerveRequest.FieldCentricFacingAngle m_faceAngleRequest = new FieldCentricFacingAngle();
  private final SwerveRequest.SwerveDriveBrake m_brakeRequest = new SwerveDriveBrake();

  public Drive(Swerve swerve) {
    m_swerve = swerve;
  }

  @Override
  public void periodic() {
    Pose2d targetPose = PathPlannerUtil.getCurrentTargetPose();

    m_currentPosePub.accept(DataLogUtil.pose2d(getPose()));
    m_desiredPathPosePub.accept(DataLogUtil.pose2d(targetPose));

    FieldUtil.getDefaultField().setSwerveRobotPose(getPose(), getModuleStates(),
        DriveConstants.kSwerveModuleTranslations);

    FieldUtil.getDefaultField().setObjectGlobalPose("Target Pose", targetPose);
  }

  @Override
  public void simulationPeriodic() {
    m_swerve.updateSimState(0.02, 12);
  }

  public void driveFieldCentric(ChassisSpeeds fieldCentricSpeeds) {
    driveFieldCentric(fieldCentricSpeeds, false);
  }

  public void driveFieldCentric(ChassisSpeeds fieldCentricSpeeds, boolean openLoop) {
    m_swerve.setControl(m_fieldCentricRequest
        .withIsOpenLoop(openLoop)
        .withVelocityX(fieldCentricSpeeds.vxMetersPerSecond)
        .withVelocityY(fieldCentricSpeeds.vyMetersPerSecond)
        .withRotationalRate(fieldCentricSpeeds.omegaRadiansPerSecond));
  }

  public void drive(ChassisSpeeds speeds) {
    drive(speeds, false);
  }

  public void drive(ChassisSpeeds speeds, boolean openLoop) {
    m_desiredSpeedsPub.accept(DataLogUtil.chassisSpeeds(speeds));

    m_swerve.setControl(m_robotCentricRequest
        .withIsOpenLoop(openLoop)
        .withVelocityX(speeds.vxMetersPerSecond)
        .withVelocityY(speeds.vyMetersPerSecond)
        .withRotationalRate(speeds.omegaRadiansPerSecond));

    m_currentStatesPub.accept(DataLogUtil.swerveModuleStates(getModuleStates()));
    // m_currentAbsoluteStatesPub.accept(DataLogUtil.swerveModuleStates(m_swerve.getAbsoluteStates()));
    m_desiredStatesPub
        .accept(DataLogUtil.swerveModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds)));
  }

  public void brake() {
    m_swerve.setControl(m_brakeRequest);
  }

  public Pose2d getPose() {
    return m_swerve.getPose2d();
  }

  public Pose3d getPose3d() {
    var pose2d = getPose();
    return new Pose3d(pose2d.getX(), pose2d.getY(), 0, m_swerve.getRotation3d());
  }

  public void resetPose(Pose2d pose) {
    m_swerve.resetPose(pose);
  }

  public void seedFieldRelative() {
    m_swerve.seedFieldRelative();
  }

  public SwerveModuleState[] getModuleStates() {
    return m_swerve.getStates();
  }

  public ChassisSpeeds getChassisSpeeds() {
    return m_swerve.getChassisSpeeds();
  }

  public Command brakeCommand() {
    return runOnce(this::brake);
  }

  public Command seedFieldRelativeCommand() {
    return Commands.runOnce(this::seedFieldRelative);
  }
}
