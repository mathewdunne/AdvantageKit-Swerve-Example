// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.NoteVisualizer;
import frc.robot.util.VisionSimNoteCallback;

public class FeederEjectCmd extends Command {

  private final Feeder m_feeder;
  private final Wrist m_wrist;
  private final Arm m_arm;
  private final VisionSimNoteCallback m_visionSimNoteCallback;

  private boolean m_showedVisualizer = false;

  /** Creates a new FeederEjectCmd. */
  public FeederEjectCmd(
      Feeder feeder, Wrist wrist, Arm arm, VisionSimNoteCallback visionSimNoteCallback) {

    m_feeder = feeder;
    m_wrist = wrist;
    m_arm = arm;
    m_visionSimNoteCallback = visionSimNoteCallback;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_feeder, m_wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_wrist.setAngleSetpoint(WristConstants.kMinAngleRad);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_wrist.atSetpoint()) {
      m_feeder.runAtVoltage(FeederConstants.kEjectVoltage);

      // Unbreak simulated beambreak
      if (m_feeder.getBeambreakBroken()) {
        m_feeder.setBeambreakUnbrokenAfterDelay();
      }

      // Show a note ejecting
      if (!m_showedVisualizer && NoteVisualizer.getHasNote()) {
        NoteVisualizer.eject(m_visionSimNoteCallback).schedule();
        m_showedVisualizer = true;
      }
    } else {
      m_feeder.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_feeder.stop();
    m_showedVisualizer = false;
    m_arm.setAngleSetpoint(ArmConstants.kStowedAngleRad);
    m_wrist.setAngleSetpoint(WristConstants.kStowedAngleRad);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
