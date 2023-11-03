package frc.lib.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.utils.CollectionUtil;

public class Swerve extends SwerveDrivetrain {

  private Pose2d m_currentPose;
  private SwerveModuleState[] m_currentStates;
  private ChassisSpeeds m_currentSpeeds;

  public Swerve(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
    super(driveTrainConstants, modules);

    m_currentPose = new Pose2d();
    m_currentStates = CollectionUtil.fillArray(new SwerveModuleState[4], SwerveModuleState::new);
    m_currentSpeeds = new ChassisSpeeds();

    registerTelemetry(state -> {
      m_currentPose = state.Pose;
      m_currentStates = state.ModuleStates;
      m_currentSpeeds = m_kinematics.toChassisSpeeds(m_currentStates);
    });
  }

  /**
  * Takes the specified location and makes it the current pose for
  * field-relative maneuvers
  *
  * @param pose Pose to make the current pose at.
  */
  public void resetPose(Pose2d pose) {
    try {
      m_stateLock.writeLock().lock();

      m_currentPose = pose;
      m_odometry.resetPosition(m_pigeon2.getRotation2d(), m_modulePositions, pose);
    } finally {
      m_stateLock.writeLock().unlock();
    }
  }

  public Pigeon2 getPigeon2() {
    return m_pigeon2;
  }

  public Pose2d getPose2d() {
    return m_currentPose;
  }

  public SwerveModuleState[] getStates() {
    return m_currentStates;
  }

  public ChassisSpeeds getChassisSpeeds() {
    return m_currentSpeeds;
  }

  public Rotation3d getRotation3d() {
    return new Rotation3d(new Quaternion(
        m_pigeon2.getQuatW().getValueAsDouble(),
        m_pigeon2.getQuatX().getValueAsDouble(),
        m_pigeon2.getQuatY().getValueAsDouble(),
        m_pigeon2.getQuatZ().getValueAsDouble()));
  }

  public SwerveDriveKinematics getKinematics() {
    return m_kinematics;
  }
}
