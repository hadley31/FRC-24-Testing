package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utils.NetworkTablePath;
import frc.robot.RobotState;

public abstract class PhotonSubsystem extends SubsystemBase implements NetworkTablePath, AutoCloseable {

  private final RobotState m_state;
  private final PhotonCamera m_camera;
  private Transform3d m_robotToCamera;
  private PhotonPipelineResult m_latestResult;
  private final StructEntry<Pose3d> m_realPoseEntry = NetworkTableInstance.getDefault()
      .getStructTopic("/Test/Entry", Pose3d.struct).getEntry(new Pose3d());
  // private final DoubleArrayPublisher m_realPoseEntry;

  public PhotonSubsystem(PhotonCamera camera, Transform3d robotToCamera, RobotState state) {
    m_camera = camera;
    m_robotToCamera = robotToCamera;
    m_state = state;
    // m_realPoseEntry = getDoubleArrayEntry("RealPose", DataLogUtil.pose3d(new Pose3d()));
  }

  @Override
  public final void periodic() {
    // Get latest result
    PhotonPipelineResult result = m_camera.getLatestResult();

    if (result != null) {
      m_latestResult = result;
      periodic(result);
    }

    var robotPose = m_state.getRobotPose3d();
    if (robotPose != null) {
      // m_realPoseEntry.accept(DataLogUtil.pose3d(getCameraPose3d(m_state.getRobotPose3d())));
      m_realPoseEntry.accept(getCameraPose3d(robotPose));
    }
  }

  public abstract void periodic(PhotonPipelineResult result);

  public PhotonCamera getCamera() {
    return m_camera;
  }

  public PhotonPipelineResult getLatestResult() {
    return m_latestResult;
  }

  public Optional<PhotonTrackedTarget> getLatestBestTarget() {
    return Optional.ofNullable(getLatestResult()).map(x -> x.getBestTarget());
  }

  public void setRobotToCamera(Transform3d transform) {
    m_robotToCamera = transform;
  }

  public Transform3d getRobotToCamera() {
    return m_robotToCamera;
  }

  public Transform3d getCameraToRobot() {
    return m_robotToCamera.inverse();
  }

  public Pose3d getCameraPose3d(Pose3d robotPose) {
    return robotPose.transformBy(getRobotToCamera());
  }

  public Pose3d getCameraPose3d(Pose2d robotPose) {
    return getCameraPose3d(new Pose3d(robotPose));
  }

  public String getName() {
    return getCamera().getName();
  }

  @Override
  public final String getPathRoot() {
    return "Cameras/" + getName();
  }

  public RobotState getState() {
    return m_state;
  }

  @Override
  public void close() {
    m_realPoseEntry.close();
  }
}
