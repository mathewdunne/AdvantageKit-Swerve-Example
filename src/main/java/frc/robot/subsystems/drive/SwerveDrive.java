// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.AimLockConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleLocation;
import frc.robot.util.GeometryUtils;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.TunablePIDController;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SwerveDrive extends SubsystemBase {
  private static final double m_maxLinearSpeed = DriveConstants.kMaxSpeedMetersPerSecond;
  private static final double m_trackWidthX =
      Units.inchesToMeters(DriveConstants.kTrackLengthInches);
  private static final double m_trackWidthY =
      Units.inchesToMeters(DriveConstants.kTrackWidthInches);
  private static final double m_driveBaseRadius =
      Math.hypot(m_trackWidthX / 2.0, m_trackWidthY / 2.0);
  private static final double m_maxAngularSpeed = m_maxLinearSpeed / m_driveBaseRadius;

  private final GyroIO m_gyroIO;
  private final GyroIOInputsAutoLogged m_gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] m_modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine m_sysId;

  // noiseless "actual" pose of the robot on the field (for vision simulation)
  @AutoLogOutput(key = "Odometry/SimRobotTrue")
  private Pose2d m_simTruePose;

  private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d m_rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] m_lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator m_poseEstimator =
      new SwerveDrivePoseEstimator(
          m_kinematics, m_rawGyroRotation, m_lastModulePositions, new Pose2d());

  // Aimbot PID controller, used for all aim to target functions
  private final PIDController m_aimLockPID;

  public SwerveDrive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.m_gyroIO = gyroIO;
    m_modules[0] = new Module(flModuleIO, ModuleLocation.FRONT_LEFT);
    m_modules[1] = new Module(frModuleIO, ModuleLocation.FRONT_RIGHT);
    m_modules[2] = new Module(blModuleIO, ModuleLocation.BACK_LEFT);
    m_modules[3] = new Module(brModuleIO, ModuleLocation.BACK_RIGHT);

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::setPose,
        () -> m_kinematics.toChassisSpeeds(getModuleStates()),
        this::runVelocity,
        new HolonomicPathFollowerConfig(
            m_maxLinearSpeed, m_driveBaseRadius, new ReplanningConfig()),
        () ->
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    m_sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                  for (int i = 0; i < 4; i++) {
                    m_modules[i].runCharacterization(voltage.in(Volts));
                  }
                },
                null,
                this));

    // set up AimLock PID
    m_aimLockPID =
        new TunablePIDController(
            AimLockConstants.kP,
            AimLockConstants.kI,
            AimLockConstants.kD,
            AimLockConstants.kToleranceRad,
            "Aimbot",
            true);
    m_aimLockPID.enableContinuousInput(-Math.PI, Math.PI);

    // True pose for vision sim
    if (Constants.kCurrentMode == Constants.Mode.SIM) {
      m_simTruePose = new Pose2d();
    }
  }

  public void periodic() {
    m_gyroIO.updateInputs(m_gyroInputs);
    Logger.processInputs("Drive/Gyro", m_gyroInputs);
    for (var module : m_modules) {
      module.periodic();
    }

    // Stop moving and empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      for (var module : m_modules) {
        module.stop();
      }
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Read wheel positions and deltas from each module
    SwerveModulePosition[] modulePositions = getModulePositions();
    SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
    for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
      moduleDeltas[moduleIndex] =
          new SwerveModulePosition(
              modulePositions[moduleIndex].distanceMeters
                  - m_lastModulePositions[moduleIndex].distanceMeters,
              modulePositions[moduleIndex].angle);
      m_lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
    }

    // Update gyro angle
    if (m_gyroInputs.connected) {
      // Use the real gyro angle
      m_rawGyroRotation = m_gyroInputs.yawPosition;
    } else {
      // Use the angle delta from the kinematics and module deltas
      Twist2d twist = m_kinematics.toTwist2d(moduleDeltas);
      m_rawGyroRotation = m_rawGyroRotation.plus(new Rotation2d(twist.dtheta));
    }

    // Apply odometry update
    m_poseEstimator.update(m_rawGyroRotation, modulePositions);

    if (Constants.kCurrentMode == Constants.Mode.SIM) {
      m_simTruePose = m_simTruePose.exp(m_kinematics.toTwist2d(moduleDeltas));
    }
  }

  /**
   * Runs the drive at the desired velocity. (with drift correction)
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    // Discretize does the same thing as the correction code below, but we may need to up the
    // driftrate
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = m_kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, m_maxLinearSpeed);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = m_modules[i].runSetpoint(setpointStates[i]);
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  // Epic correction for drift
  // Sources:
  // https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/5
  // https://github.com/Team7520/UltimateSwerveBase
  // DriftRate was added by 4920, they used 4
  private static ChassisSpeeds correctForDynamics(ChassisSpeeds originalSpeeds, double driftRate) {
    Pose2d futureRobotPose =
        new Pose2d(
            originalSpeeds.vxMetersPerSecond * Constants.kLoopPeriodSecs,
            originalSpeeds.vyMetersPerSecond * Constants.kLoopPeriodSecs,
            Rotation2d.fromRadians(
                originalSpeeds.omegaRadiansPerSecond * Constants.kLoopPeriodSecs * driftRate));
    Twist2d twistForPose = GeometryUtils.log(futureRobotPose);
    ChassisSpeeds updatedSpeeds =
        new ChassisSpeeds(
            twistForPose.dx / Constants.kLoopPeriodSecs,
            twistForPose.dy / Constants.kLoopPeriodSecs,
            twistForPose.dtheta / Constants.kLoopPeriodSecs);
    return updatedSpeeds;
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    m_kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysId.dynamic(direction);
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = m_modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = m_modules[i].getPosition();
    }
    return states;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    m_poseEstimator.resetPosition(m_rawGyroRotation, getModulePositions(), pose);

    if (Constants.kCurrentMode == Constants.Mode.SIM) {
      m_simTruePose = pose;
    }
  }

  /** Resets the current odometry pose (0, 0) */
  public void resetOdometry() {
    setPose(new Pose2d());
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionPose The pose of the robot as measured by the vision camera.
   * @param timestamp The timestamp of the vision measurement in seconds.
   * @param stDevs The standard deviations of the vision measurement.
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> stDevs) {
    m_poseEstimator.addVisionMeasurement(visionPose, timestamp, stDevs);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return m_maxLinearSpeed;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return m_maxAngularSpeed;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(m_trackWidthX / 2.0, m_trackWidthY / 2.0),
      new Translation2d(m_trackWidthX / 2.0, -m_trackWidthY / 2.0),
      new Translation2d(-m_trackWidthX / 2.0, m_trackWidthY / 2.0),
      new Translation2d(-m_trackWidthX / 2.0, -m_trackWidthY / 2.0)
    };
  }

  /** Returns the true pose of the robot on the field (for vision simulation). */
  public Pose2d getSimTruePose() {
    return m_simTruePose;
  }

  /** Returns the AimLock PID controller used for aiming to a desired angle */
  public PIDController getAimLockPID() {
    return m_aimLockPID;
  }

  /** Gets whether or not the AimLock PID is at its setpoint */
  public boolean aimedAtSetpoint() {
    return m_aimLockPID.atSetpoint();
  }
}
