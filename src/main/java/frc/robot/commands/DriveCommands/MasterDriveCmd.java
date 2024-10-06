// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AimDriveMode;
import frc.robot.Constants.BaseDriveMode;
import frc.robot.subsystems.drive.SwerveSubsystem;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class MasterDriveCmd extends Command {

  private final SwerveSubsystem m_swerveSubsystem;
  private final DoubleSupplier m_xSupplier;
  private final DoubleSupplier m_ySupplier;
  private final DoubleSupplier m_omegaSupplier;

  @AutoLogOutput(key = "Drive/BaseMode")
  private BaseDriveMode m_baseDriveMode;

  @AutoLogOutput(key = "Drive/AimMode")
  private AimDriveMode m_aimDriveMode;

  private Command m_currentCommand;

  /** Creates a new MasterDriveCommand. */
  public MasterDriveCmd(
      SwerveSubsystem swerveSubsystem,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    m_swerveSubsystem = swerveSubsystem;
    m_xSupplier = xSupplier;
    m_ySupplier = ySupplier;
    m_omegaSupplier = omegaSupplier;

    m_baseDriveMode = BaseDriveMode.FIELD_ORIENTED;
    m_aimDriveMode = AimDriveMode.NONE;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setDriveCommand();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_currentCommand != null) {
      m_currentCommand.schedule();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (m_currentCommand != null) {
      m_currentCommand.end(interrupted);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private void setDriveCommand() {
    if (m_aimDriveMode == AimDriveMode.NONE) {
      if (m_baseDriveMode == BaseDriveMode.FIELD_ORIENTED) {
        m_currentCommand =
            new FieldOrientedDriveCmd(m_swerveSubsystem, m_xSupplier, m_ySupplier, m_omegaSupplier);
      } else {
        m_currentCommand =
            new RobotOrientedDriveCmd(m_swerveSubsystem, m_xSupplier, m_ySupplier, m_omegaSupplier);
      }
    } else {
      // Add aim drive command here
    }
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
    setDriveCommand();
  }
}
