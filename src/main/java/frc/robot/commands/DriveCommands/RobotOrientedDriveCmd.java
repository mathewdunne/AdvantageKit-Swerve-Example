// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.drive.SwerveSubsystem;
import java.util.function.DoubleSupplier;

public class RobotOrientedDriveCmd extends Command {

  private final SwerveSubsystem m_swerveSubsystem;
  private final DoubleSupplier m_xSupplier;
  private final DoubleSupplier m_ySupplier;
  private final DoubleSupplier m_omegaSupplier;

  /** Creates a new SwerveJoystickCommand. */
  public RobotOrientedDriveCmd(
      SwerveSubsystem swerveSubsystem,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    m_swerveSubsystem = swerveSubsystem;
    m_xSupplier = xSupplier;
    m_ySupplier = ySupplier;
    m_omegaSupplier = omegaSupplier;

    // Don't call addReqirements here because the master command will do that
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Apply deadband
    double linearMagnitude =
        MathUtil.applyDeadband(
            Math.hypot(m_xSupplier.getAsDouble(), m_ySupplier.getAsDouble()),
            ControllerConstants.kDriverControllerDeadband);
    Rotation2d linearDirection =
        new Rotation2d(m_xSupplier.getAsDouble(), m_ySupplier.getAsDouble());
    double omega =
        MathUtil.applyDeadband(
            m_omegaSupplier.getAsDouble(), ControllerConstants.kDriverControllerDeadband);

    // Square values
    linearMagnitude = linearMagnitude * linearMagnitude;
    omega = Math.copySign(omega * omega, omega);

    // Calcaulate new linear velocity
    Translation2d linearVelocity =
        new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
            .getTranslation();

    // Send command
    m_swerveSubsystem.runVelocity(
        new ChassisSpeeds(
            linearVelocity.getX() * m_swerveSubsystem.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * m_swerveSubsystem.getMaxLinearSpeedMetersPerSec(),
            omega * m_swerveSubsystem.getMaxAngularSpeedRadPerSec()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
