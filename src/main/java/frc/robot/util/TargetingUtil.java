package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.AimDriveMode;
import frc.robot.Constants.VisionConstants;
import org.littletonrobotics.junction.Logger;

public class TargetingUtil {
  private static double calculateAngleToTarget(Pose2d robotPose, Translation2d targetPosition) {
    return Math.atan2(
        targetPosition.getY() - robotPose.getTranslation().getY(),
        targetPosition.getX() - robotPose.getTranslation().getX());
  }

  /*
   * Returns the angle to the target from the robot's current pose.
   */
  public static double getAngleToTarget(Pose2d robotPose, AimDriveMode driveMode) {
    boolean isRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    double angle = 0.0;
    switch (driveMode) {
      case AIM_SPEAKER:
        if (isRed) {
          angle =
              calculateAngleToTarget(
                  robotPose,
                  VisionConstants.kTagLayout.getTagPose(4).get().toPose2d().getTranslation());
        } else {
          angle =
              calculateAngleToTarget(
                  robotPose,
                  VisionConstants.kTagLayout.getTagPose(7).get().toPose2d().getTranslation());
        }
        break;
      case FACE_AMP:
        angle = Math.PI / 2;
        break;
      case FACE_SOURCE:
        if (isRed) {
          angle = Math.PI / 3;
        } else {
          angle = 2 * Math.PI / 3;
        }
        break;
      case FACE_FORWARD:
        if (isRed) {
          angle = Math.PI;
        } else {
          angle = 0;
        }
        break;
      case FACE_BACKWARD:
        if (isRed) {
          angle = 0;
        } else {
          angle = Math.PI;
        }
        break;
      case LOB_PASS:
        if (isRed) {
          angle = calculateAngleToTarget(robotPose, new Translation2d(14.75, 7.0));
        } else {
          angle = calculateAngleToTarget(robotPose, new Translation2d(1.0, 7.0));
        }
        break;
      default:
        angle = 0.0;
        break;
    }
    Logger.recordOutput("Odometry/AimLockSetpoint", angle);
    return angle;
  }
}
