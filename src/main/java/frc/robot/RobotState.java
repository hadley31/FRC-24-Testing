package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

/**
 * Defines the position of every mechanism on the robot
 */
public class RobotState {

  private Pose3d m_robotPose;

  public void updateRobotPose(Pose2d pose) {
    m_robotPose = new Pose3d(pose);
  }

  public Pose2d getRobotPose() {
    return getRobotPose3d().toPose2d();
  }

  public Pose3d getRobotPose3d() {
    return m_robotPose;
  }
}
