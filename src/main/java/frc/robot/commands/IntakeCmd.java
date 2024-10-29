// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Robot;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.wrist.Wrist;

public class IntakeCmd extends Command {
  /** Creates a new IntakeCmd. */
  private final Intake m_intake;

  private final Feeder m_feeder;
  private final Wrist m_wrist;
  private final Arm m_arm;

  public IntakeCmd(Intake intake, Feeder feeder, Wrist wrist, Arm arm) {
    m_intake = intake;
    m_feeder = feeder;
    m_wrist = wrist;
    m_arm = arm;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake, m_feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.runAtVoltage(IntakeConstants.kIntakeVoltage);
    m_feeder.runAtVoltage(FeederConstants.kFeedVoltage);

    // Simulate a note being intaked by breaking the beambreak after a delay
    if (Robot.isSimulation()) {
      m_feeder.setBeambreakBrokenAfterDelay();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
    m_feeder.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_feeder.getBeambreakBroken() || !m_wrist.isStowed() || !m_arm.isStowed();
  }
}
