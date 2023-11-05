package frc.robot.subsystems.vision;

import java.util.function.Consumer;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import frc.robot.RobotState;

public class AprilTagCamera extends PhotonSubsystem {

  private final PhotonPoseEstimator m_estimator;
  private Consumer<EstimatedRobotPose> m_poseConsumer;
  private final Vector<N3> m_stdDeviations;
  // private final ProtobufEntry<Pose3d> m_estimatedPoseEntry = getProtobufTopic("Estimated Pose", Pose3d.proto);

  public AprilTagCamera(PhotonCamera camera, Transform3d robotToCamera, AprilTagFieldLayout layout, RobotState state,
      Vector<N3> stdDeviations) {
    super(camera, robotToCamera, state);
    m_estimator = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCamera);
    m_stdDeviations = stdDeviations;
  }

  public AprilTagCamera(PhotonCamera camera, Transform3d robotToCamera, AprilTagFieldLayout layout, RobotState state) {
    this(camera, robotToCamera, layout, state, VecBuilder.fill(0.1, 0.1, 1.0));
  }

  @Override
  public void periodic(PhotonPipelineResult result) {
    var poseResult = m_estimator.update(result);

    if (poseResult.isEmpty()) {
      return;
    }

    if (m_poseConsumer != null) {
      m_poseConsumer.accept(poseResult.get());
    }

    // m_estimatedPoseEntry.accept(poseResult.get().estimatedPose);
  }

  @Override
  public void setRobotToCamera(Transform3d transform) {
    super.setRobotToCamera(transform);
    m_estimator.setRobotToCameraTransform(transform);
  }

  public void setPoseListener(Consumer<EstimatedRobotPose> consumer) {
    this.m_poseConsumer = consumer;
  }

  public Vector<N3> getStdDeviations() {
    return m_stdDeviations;
  }
}
