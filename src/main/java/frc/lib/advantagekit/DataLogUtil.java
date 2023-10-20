package frc.lib.advantagekit;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class DataLogUtil {
  /**
   * 
   * @param pose
   * @return [pose.x, pose.y, pose.degrees]
   */
  public static double[] pose2d(Pose2d... poses) {
    double[] result = new double[poses.length * 3];

    for (int i = 0; i < poses.length; i++) {
      result[i * 3] = poses[i].getX();
      result[i * 3 + 1] = poses[i].getY();
      result[i * 3 + 2] = poses[i].getRotation().getRadians();
    }

    return result;
  }

  public static double[] pose3d(Pose3d... poses) {
    double[] result = new double[poses.length * 7];

    for (int i = 0; i < poses.length; i++) {
      result[i * 7] = poses[i].getX();
      result[i * 7 + 1] = poses[i].getY();
      result[i * 7 + 2] = poses[i].getZ();
      result[i * 7 + 3] = poses[i].getRotation().getQuaternion().getW();
      result[i * 7 + 4] = poses[i].getRotation().getQuaternion().getX();
      result[i * 7 + 5] = poses[i].getRotation().getQuaternion().getY();
      result[i * 7 + 6] = poses[i].getRotation().getQuaternion().getZ();
    }

    return result;
  }

  public static double[] swerveModuleStates(SwerveModuleState... states) {
    double[] result = new double[states.length * 2];

    for (int i = 0; i < states.length; ++i) {
      result[i * 2] = states[i].angle.getRadians();
      result[i * 2 + 1] = states[i].speedMetersPerSecond;
    }

    return result;
  }

  public static double[] chassisSpeeds(ChassisSpeeds speeds) {
    return new double[] {
        speeds.vxMetersPerSecond,
        speeds.vyMetersPerSecond,
        speeds.omegaRadiansPerSecond
    };
  }
}
