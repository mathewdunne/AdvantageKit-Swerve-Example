// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FeederConstants;
import frc.robot.Robot;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.NoteVisualizer;
import java.util.function.Supplier;

public class FeederShootCmd extends Command {

  private final Feeder m_feeder;
  private final Shooter m_shooter;
  private final Wrist m_wrist;
  private final Supplier<Boolean> m_swerveAtSetpointSupplier;

  /** Creates a new FeederShootCmd. */
  public FeederShootCmd(
      Feeder feeder, Shooter shooter, Wrist wrist, Supplier<Boolean> swerveAtSetpointSupplier) {

    m_feeder = feeder;
    m_shooter = shooter;
    m_wrist = wrist;
    m_swerveAtSetpointSupplier = swerveAtSetpointSupplier;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_shooter.atSetpoint() && m_wrist.atSetpoint() && m_swerveAtSetpointSupplier.get()) {
      m_feeder.runAtVoltage(FeederConstants.kFeedVoltage);

      // Simulate a note being shot by un-breaking the beambreak after a delay
      if (Robot.isSimulation()) {
        m_feeder.setBeambreakUnbrokenAfterDelay();
      }
    } else {
      m_feeder.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_feeder.stop();
    if (!m_feeder.getBeambreakBroken() && NoteVisualizer.getHasNote()) {
      NoteVisualizer.shoot().schedule();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_feeder.getBeambreakBroken();
  }
}
