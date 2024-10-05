// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.SwerveDrive;
import java.util.function.DoubleSupplier;

public class FieldOrientedDriveCommand extends Command {

  private final SwerveDrive m_swerveSubsystem;
  private final DoubleSupplier m_xSupplier;
  private final DoubleSupplier m_ySupplier;
  private final DoubleSupplier m_omegaSupplier;

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

    addRequirements(m_swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
