package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode kCurrentMode = Mode.SIM;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final double kLoopPeriodSecs = 0.02;

  public static final double kCloseShotThresholdMeters = 3.1;

  public static final class ControllerConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriverControllerDeadband = 0.1;

    public static final int kOperatorControllerPort = 1;
  }

  public static final class DriveConstants {
    public static final double kTrackWidthInches = 28.0;
    public static final double kTrackLengthInches = 28.0;
    public static final double kWheelDiameterInches = 4.0;
    public static final double kSdrive = 0.0;
    public static final double kVdrive = 0.0;
    public static final double kPdrive = 1.0;
    public static final double kIdrive = 0.0;
    public static final double kDdrive = 0.0;
    public static final double kPturn = 1.0;
    public static final double kIturn = 0.0;
    public static final double kDturn = 0.0;
    public static final double kMaxSpeedMetersPerSecond = 4.5;
  }

  public static enum ModuleLocation {
    FRONT_LEFT,
    FRONT_RIGHT,
    BACK_LEFT,
    BACK_RIGHT
  }

  public static final class SwerveModuleConstants {
    public static final int kFrontLeftDriveMotorPort = 0;
    public static final int kFrontLeftTurningMotorPort = 1;
    public static final int kFrontLeftEncoderPort = 0;
    public static final double kFrontLeftEncoderOffset = 0.0;

    public static final int kFrontRightDriveMotorPort = 2;
    public static final int kFrontRightTurningMotorPort = 3;
    public static final int kFrontRightEncoderPort = 1;
    public static final double kFrontRightEncoderOffset = 0.0;

    public static final int kBackLeftDriveMotorPort = 4;
    public static final int kBackLeftTurningMotorPort = 5;
    public static final int kBackLeftEncoderPort = 2;
    public static final double kBackLeftEncoderOffset = 0.0;

    public static final int kBackRightDriveMotorPort = 6;
    public static final int kBackRightTurningMotorPort = 7;
    public static final int kBackRightEncoderPort = 3;
    public static final double kBackRightEncoderOffset = 0.0;

    // Gear ratios for SDS MK4i L2, adjust as necessary
    public static final double kDriveGearRatio = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
    public static final double kTurnGearRatio = 150.0 / 7.0;

    public static final int kDriveCurrentLimit = 40;
    public static final int kTurnCurrentLimit = 30;
  }

  public static final class ShooterConstants {
    public static final double kS1 = 0.0;
    public static final double kV1 = 0.0;
    public static final double kP1 = 1.0;
    public static final double kI1 = 0.0;
    public static final double kD1 = 0.0;

    public static final double kS2 = 0.0;
    public static final double kV2 = 0.0;
    public static final double kP2 = 1.0;
    public static final double kI2 = 0.0;
    public static final double kD2 = 0.0;
    public static final double kToleranceRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(100);

    public static final double kIdleVelocityRPM = 2800;
    public static final double kShootCloseVelocityRPM = 5000;
    public static final double kShootFarVelocityRPM = 3500;
    public static final double kPassCloseVelocityRPM = 2800;
    public static final double kPassFarVelocityRPM = 3700;
    public static final double kPassVelocityRPM = 3300;
    public static final double[] kAmpVoltages = {0.4 * 12, 0.2 * 12};

    public static final int kLeaderMotorPort = 10;
    public static final int kFollowerMotorPort = 11;
    public static final int kCurrentLimit = 30;
  }

  public static final class FeederConstants {
    public static final double kFeedVoltage = 12.0;
    public static final double kIntakeVoltage = 12.0;
    public static final double kEjectVoltage = -12.0;

    public static final int kMotorPort = 15;
    public static final int kCurrentLimit = 30;

    public static final int kBeambreakPort = 0;
  }

  public static final class IntakeConstants {
    public static final double kIntakeVoltage = 12.0;
    public static final double kEjectVoltage = -12.0;

    public static final int kMotorPort = 16;
    public static final int kCurrentLimit = 30;
    public static final double kCurrentThreshold = 18.0;

    // Location of the intake relative to the robot
    public static final Transform3d kRobotToIntake = new Transform3d(-0.35, 0, 0, new Rotation3d());
  }

  public static enum BaseDriveMode {
    FIELD_ORIENTED,
    ROBOT_ORIENTED,
  }

  public static enum AimDriveMode {
    NONE,
    AIM_SPEAKER,
    AIM_NOTE,
    FACE_FORWARD,
    FACE_BACKWARD,
    FACE_AMP,
    FACE_SOURCE,
    LOB_PASS,
    TRACK_NOTE,
  }

  public static class AimLockConstants {
    public static final double kP = 10;
    public static final double kI = 0.0;
    public static final double kD = 0.5;
    public static final double kToleranceRad = Units.degreesToRadians(2);
  }

  public static class VisionConstants {
    public static final String kApriltagCameraName = "copperShooterCamera";
    public static final Transform3d kRobotToApriltagCam =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(11.56 + 0.39 + 16.0), 0.0, Units.inchesToMeters(13.876 + 1.9)),
            new Rotation3d(0, Units.degreesToRadians(-15), 0));
    public static final String kIntakeCameraName = "silverIntakeCamera";
    public static final Transform3d kRobotToIntakeCam =
        new Transform3d(
            new Translation3d(-0.24, -0.2, 0.5),
            new Rotation3d(0, Units.degreesToRadians(15), Units.degreesToRadians(180)));

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout =
        AprilTagFields.kDefaultField.loadAprilTagLayoutField();

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  }

  public static class ArmConstants {
    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kG = 0.0;
    public static final double kP = 1.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kToleranceRad = Units.degreesToRadians(2);
    public static final double kMaxAccelerationRadPerSec2 = 120;

    public static final int kMotorPort = 12;
    public static final int kCurrentLimit = 40;
    public static final int kAbsoluteEncoderOffset = 0;

    public static final double kGearRatio = 240;
    public static final double kLengthMeters =
        Units.inchesToMeters(17.5); // between the arm pivot and the wrist pivot
    public static final double kMOIkgm2 = 2.767; // moment of inertia (from CAD)
    public static final double kMinAngleRad =
        Units.degreesToRadians(6); // 0 rad is horizontal, hardstop is slightly above
    public static final double kMaxAngleRad = Units.degreesToRadians(85); // max is almost vertical
    public static final double kStowedAngleRad = kMinAngleRad;
    public static final double kAmpAngleRad = Units.degreesToRadians(85);
  }

  public static class WristConstants {
    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kG = 0.0;
    public static final double kP = 1.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kToleranceRad = Units.degreesToRadians(1);
    public static final double kMaxAccelerationRadPerSec2 = 50;

    public static final int kMotorPort = 13;
    public static final int kCurrentLimit = 40;
    public static final int kAbsoluteEncoderOffset = 0;

    public static final double kGearRatio = 40;
    public static final double kLengthMeters =
        Units.inchesToMeters(7.0); // between the wrist pivot and the tip of the shooter end
    public static final double kMOIkgm2 = 0.329; // moment of inertia (from CAD)

    // For these angles, they are relative to the arm, with 0 degrees being parallel to the arm
    public static final double kMinAngleRad = Units.degreesToRadians(45);
    public static final double kMaxAngleRad = Units.degreesToRadians(135);
    public static final double kStowedAngleRad = Units.degreesToRadians(67);
    public static final double kAmpAngleRad = Units.degreesToRadians(60);
    public static final double kSubwooferAngleRad = Units.degreesToRadians(63);
  }
}
