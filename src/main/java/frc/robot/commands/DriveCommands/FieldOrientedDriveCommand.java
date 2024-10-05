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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.drive.SwerveDrive;

import java.util.function.DoubleSupplier;

public class FieldOrientedDriveCommand extends Command {

  private final SwerveDrive m_swerveSubsystem;
  private final DoubleSupplier m_xSupplier;
  private final DoubleSupplier m_ySupplier;
  private final DoubleSupplier m_omegaSupplier;
  private final double DEADBAND;

  /** Creates a new SwerveJoystickCommand. */
  public FieldOrientedDriveCommand(
      SwerveDrive swerveSubsystem,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerveSubsystem = swerveSubsystem;
    m_xSupplier = xSupplier;
    m_ySupplier = ySupplier;
    m_omegaSupplier = omegaSupplier;
    DEADBAND = ControllerConstants.kDriverControllerDeadband;

    addRequirements(m_swerveSubsystem);
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
            Math.hypot(m_xSupplier.getAsDouble(), m_ySupplier.getAsDouble()), DEADBAND);
    Rotation2d linearDirection =
        new Rotation2d(m_xSupplier.getAsDouble(), m_ySupplier.getAsDouble());
    double omega = MathUtil.applyDeadband(m_omegaSupplier.getAsDouble(), DEADBAND);

    // Square values
    linearMagnitude = linearMagnitude * linearMagnitude;
    omega = Math.copySign(omega * omega, omega);

    // Calcaulate new linear velocity
    Translation2d linearVelocity =
        new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
            .getTranslation();

    // Convert to field relative speeds & send command
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    m_swerveSubsystem.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            linearVelocity.getX() * m_swerveSubsystem.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * m_swerveSubsystem.getMaxLinearSpeedMetersPerSec(),
            omega * m_swerveSubsystem.getMaxAngularSpeedRadPerSec(),
            isFlipped
                ? m_swerveSubsystem.getRotation().plus(new Rotation2d(Math.PI))
                : m_swerveSubsystem.getRotation()));
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
