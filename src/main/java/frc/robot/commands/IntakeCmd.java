// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Robot;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.NoteVisualizer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class IntakeCmd extends Command {
  /** Creates a new IntakeCmd. */
  private final Intake m_intake;

  private final Feeder m_feeder;
  private final Wrist m_wrist;
  private final Arm m_arm;
  private final CommandGenericHID m_controller;
  private final Supplier<Pose2d> m_poseSupplier;

  private double m_timer = 0;
  private final double m_rumbleDuration = 0.2;
  private final double m_rumbleCooldown = 3.0;

  public IntakeCmd(
      Intake intake,
      Feeder feeder,
      Wrist wrist,
      Arm arm,
      CommandGenericHID controller,
      Supplier<Pose2d> poseSupplier) {
    m_intake = intake;
    m_feeder = feeder;
    m_wrist = wrist;
    m_arm = arm;
    m_controller = controller;
    m_poseSupplier = poseSupplier;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake, m_feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.runAtVoltage(IntakeConstants.kIntakeVoltage);
    m_feeder.runAtVoltage(FeederConstants.kIntakeVoltage);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_intake.hasNote() && Timer.getFPGATimestamp() - m_timer >= m_rumbleCooldown) {
      // if there is a note and cooldown time has passed, start rumble timer
      m_timer = Timer.getFPGATimestamp();
    } else if (m_intake.hasNote() && Timer.getFPGATimestamp() - m_timer < m_rumbleDuration) {
      // if there is a note and we're within rumble time, rumble
      m_controller.getHID().setRumble(RumbleType.kBothRumble, 1);
    } else {
      // turn off rumble
      m_controller.getHID().setRumble(RumbleType.kBothRumble, 0);
    }

    // only allow sim intake if the robot is near a note
    if (Robot.isSimulation()) {
      Pose3d intakePose =
          new Pose3d(m_poseSupplier.get()).transformBy(IntakeConstants.kRobotToIntake);
      boolean nearNote = false;
      for (int i = 0; i < NoteVisualizer.getFieldNotes().size(); i++) {
        Translation2d notePose = NoteVisualizer.getFieldNotes().get(i);
        if (notePose != null
            && intakePose.getTranslation().toTranslation2d().getDistance(notePose) < 0.5) {
          nearNote = true;
          // Simulate a note being intaked by breaking the beambreak after a delay
          m_feeder.setBeambreakBrokenAfterDelay();

          // remove the note from the field
          NoteVisualizer.takeFieldNote(i);
          break;
        }
      }
      Logger.recordOutput("Intake/SimNearNote", nearNote);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
    m_feeder.stop();
    if (m_feeder.getBeambreakBroken()) {
      NoteVisualizer.setHasNote(true);
    }
    Logger.recordOutput("Intake/SimNearNote", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_feeder.getBeambreakBroken() || !m_wrist.isStowed() || !m_arm.isStowed();
  }
}
