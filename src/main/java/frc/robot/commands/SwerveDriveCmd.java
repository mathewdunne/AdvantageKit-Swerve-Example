// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AimDriveMode;
import frc.robot.Constants.BaseDriveMode;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.drive.SwerveDrive;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class SwerveDriveCmd extends Command {

  private final SwerveDrive m_swerveDrive;
  private final DoubleSupplier m_xSupplier;
  private final DoubleSupplier m_ySupplier;
  private final DoubleSupplier m_omegaSupplier;

  @AutoLogOutput(key = "Drive/BaseMode")
  private BaseDriveMode m_baseDriveMode;

  @AutoLogOutput(key = "Drive/AimMode")
  private AimDriveMode m_aimDriveMode;

  /** Creates a new MasterDriveCommand. */
  public SwerveDriveCmd(
      SwerveDrive SwerveDrive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    m_swerveDrive = SwerveDrive;
    m_xSupplier = xSupplier;
    m_ySupplier = ySupplier;
    m_omegaSupplier = omegaSupplier;

    m_baseDriveMode = BaseDriveMode.FIELD_ORIENTED;
    m_aimDriveMode = AimDriveMode.NONE;

    addRequirements(m_swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Cancel aim lock if the driver is trying to turn manually
    if (m_aimDriveMode != AimDriveMode.NONE && Math.abs(m_omegaSupplier.getAsDouble()) > 0.1) {
      m_aimDriveMode = AimDriveMode.NONE;
    }

    // Get joystick inputs
    double xInput = m_xSupplier.getAsDouble();
    double yInput = m_ySupplier.getAsDouble();
    double omegaInput = m_omegaSupplier.getAsDouble();

    // Apply deadband and square the magnitude for finer control at low speeds
    double magnitude = Math.hypot(xInput, yInput);
    double direction = Math.atan2(yInput, xInput);
    double adjustedMagnitude =
        MathUtil.applyDeadband(magnitude, ControllerConstants.kDriverControllerDeadband);
    adjustedMagnitude = adjustedMagnitude * adjustedMagnitude;

    // Reconstruct the adjusted x and y values based on the adjusted magnitude
    double adjustedX = adjustedMagnitude * Math.cos(direction);
    double adjustedY = adjustedMagnitude * Math.sin(direction);

    // Apply deadband and square the omega input
    double omega =
        MathUtil.applyDeadband(omegaInput, ControllerConstants.kDriverControllerDeadband);
    omega = Math.copySign(omega * omega, omega);

    // Scale the velocities to the robot's maximum speed
    double maxLinearSpeed = m_swerveDrive.getMaxLinearSpeedMetersPerSec();
    double maxAngularSpeed = m_swerveDrive.getMaxAngularSpeedRadPerSec();

    double vx = adjustedX * maxLinearSpeed;
    double vy = adjustedY * maxLinearSpeed;

    double omegaRadiansPerSec;

    // Determine omega based on aim mode
    if (m_aimDriveMode == AimDriveMode.NONE || true) {
      // Manual control of omega
      omegaRadiansPerSec = omega * maxAngularSpeed;
    }

    // Determine if the robot orientation should be flipped based on alliance color
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    Rotation2d robotRotation = m_swerveDrive.getRotation();
    if (isFlipped) {
      robotRotation = robotRotation.plus(new Rotation2d(Math.PI));
    }

    // Create chassis speeds based on the base drive mode
    ChassisSpeeds chassisSpeeds;
    if (m_baseDriveMode == BaseDriveMode.FIELD_ORIENTED) {
      // Field-oriented control
      chassisSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omegaRadiansPerSec, robotRotation);
    } else {
      // Robot-oriented control
      // Negative x so intake is forward
      chassisSpeeds = new ChassisSpeeds(-vx, vy, omegaRadiansPerSec);
    }

    // Send the calculated speeds to the swerve subsystem
    m_swerveDrive.runVelocity(chassisSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  /*
   * Toggles the base drive mode between field-oriented and robot-oriented.
   */
  public void toggleBaseDriveMode() {
    if (m_baseDriveMode == BaseDriveMode.FIELD_ORIENTED) {
      m_baseDriveMode = BaseDriveMode.ROBOT_ORIENTED;
    } else {
      m_baseDriveMode = BaseDriveMode.FIELD_ORIENTED;
    }
  }

  /*
   * Sets to robot oriented
   */
  public void setRobotOriented() {
    m_baseDriveMode = BaseDriveMode.ROBOT_ORIENTED;
  }

  /*
   * Sets to field oriented
   */
  public void setFieldOriented() {
    m_baseDriveMode = BaseDriveMode.FIELD_ORIENTED;
  }

  /*
   * Turn on one of the aim lock modes
   */
  public void setAimDriveMode(AimDriveMode mode) {
    m_aimDriveMode = mode;
  }
}
