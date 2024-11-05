package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.Constants.AimDriveMode;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.WristConstants;
import java.util.Optional;
import java.util.function.Supplier;
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

  /*
   * Returns the distance to the speaker from the robot's current pose.
   */
  public static double getDistanceToSpeaker(Pose2d robotPose) {
    double distance =
        robotPose
            .getTranslation()
            .getDistance(
                VisionConstants.kTagLayout.getTagPose(4).get().toPose2d().getTranslation());
    Logger.recordOutput("Odometry/DistanceToSpeaker", distance);
    return distance;
  }

  /*
   * Returns the angle of the wrist in radians based on the distance to the speaker.
   */
  public static double getWristAngle(double distanceToSpeaker) {
    double angle = 0.0;
    if (distanceToSpeaker < Constants.kCloseShotThresholdMeters) {
      // close shot
      angle = 0.491 * Math.atan(1 / distanceToSpeaker) + 0.586;
    } else {
      // far shot
      angle = 0.491 * Math.atan(1 / distanceToSpeaker) + 0.586;
    }
    angle = MathUtil.clamp(angle, WristConstants.kMinAngleRad, WristConstants.kMaxAngleRad);
    Logger.recordOutput("Wrist/AngleSetpoint", angle);
    return angle;
  }

  /*
   * Returns the shooter velocity in RPM based on the distance to the speaker.
   */
  public static double getShooterRPM(double distanceToSpeaker) {
    if (distanceToSpeaker < Constants.kCloseShotThresholdMeters) {
      // close shot
      return ShooterConstants.kShootCloseVelocityRPM;
    } else {
      // far shot
      return ShooterConstants.kShootFarVelocityRPM;
    }
  }

  /*
   * Returns the error angle in radians to the closest note
   */
  public static double getAngleToNote(Supplier<Optional<Translation2d>> getClosestNote) {
    Optional<Translation2d> closestNote = getClosestNote.get();
    if (closestNote.isEmpty()) return 0.0;

    double pitch = closestNote.get().getX();
    double yaw = closestNote.get().getY();

    // return 0 if the note is too far away to prevent weird behaviour when a close note is right
    // under the bumper
    if (pitch > 5.0) return 0.0;
    double angle = yaw + 0.391 * pitch + 0.478;
    double radians = Units.degreesToRadians(angle);
    Logger.recordOutput("Vision/AngleToNote", radians);
    return radians;
  }
}
