package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class PhotonSubsystem extends SubsystemBase {

  private Transform3d m_robotToCamera;

  @Override
  public final void periodic() {
    // Get latest result
    periodic(null);
  }

  public abstract void periodic(Object result);

  public void setRobotToCamera(Transform3d transform) {
    m_robotToCamera = transform;
  }

  public Transform3d getRobotToCamera() {
    return m_robotToCamera;
  }

  public Transform3d getCameraToRobot() {
    return m_robotToCamera.inverse();
  }
}
