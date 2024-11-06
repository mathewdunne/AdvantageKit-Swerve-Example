// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.Robot;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.NoteVisualizer;

public class AutoSubwooferShotCmd extends Command {

  private final Feeder m_feeder;
  private final Wrist m_wrist;

  double m_timerStart = 0;
  boolean m_timerFinished = false;
  double m_delay = 0.2;

  /** Creates a new AutoSubwooferShot. */
  public AutoSubwooferShotCmd(Feeder feeder, Wrist wrist) {
    m_feeder = feeder;
    m_wrist = wrist;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_wrist, m_feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_wrist.setAngleSetpoint(WristConstants.kSubwooferAngleRad);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // wait for wrist to reach setpoint
    if (m_wrist.atSetpoint()) {
      if (m_timerStart == 0) {
        m_timerStart = Timer.getFPGATimestamp();
      } else if (Timer.getFPGATimestamp() - m_timerStart > m_delay) {
        m_timerFinished = true;
      }
    }

    // run feeder
    if (m_timerFinished) {
      m_feeder.runAtVoltage(FeederConstants.kFeedVoltage);

      // Simulate a note being shot by un-breaking the beambreak after a delay
      if (Robot.isSimulation()) {
        m_feeder.setBeambreakUnbrokenAfterDelay();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_wrist.setAngleSetpoint(WristConstants.kStowedAngleRad);
    m_feeder.stop();
    if (!m_feeder.getBeambreakBroken() && NoteVisualizer.getHasNote()) {
      NoteVisualizer.shoot().schedule();
    }

    // reset timer
    m_timerStart = 0;
    m_timerFinished = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_feeder.getBeambreakBroken();
  }
}
