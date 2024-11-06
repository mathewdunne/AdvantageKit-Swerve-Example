// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AimDriveMode;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.Robot;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.NoteVisualizer;
import frc.robot.util.TargetingUtil;

public class AutoSpeakerShotCmd extends Command {

  private final SwerveDrive m_swerveDrive;
  private final Wrist m_wrist;
  private final Shooter m_shooter;
  private final Feeder m_feeder;
  private final Intake m_intake;

  // Delay for stuff to settle before shooting
  double m_dwellStart = 0;
  boolean m_dwellFinished = false;
  double m_dwellDuration = 0.2;

  // Timeout if we don't have a note
  double m_noNoteDwellStart = 0;
  double m_noNoteDwellDuration = 2.0;

  boolean m_hadNote = false;

  /** Creates a new AutoSpeakerShotCmd. */
  public AutoSpeakerShotCmd(
      SwerveDrive swerveDrive, Wrist wrist, Shooter shooter, Feeder feeder, Intake intake) {
    m_swerveDrive = swerveDrive;
    m_wrist = wrist;
    m_shooter = shooter;
    m_feeder = feeder;
    m_intake = intake;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_wrist, m_swerveDrive, m_feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Check for note
    if (m_feeder.getBeambreakBroken()) {
      m_hadNote = true;
    }
    // Aim the chassis
    double targetAngle =
        TargetingUtil.getAngleToTarget(m_swerveDrive.getPose(), AimDriveMode.AIM_SPEAKER);
    double currentAngle = m_swerveDrive.getPose().getRotation().getRadians();
    double omegaRadiansPerSec = m_swerveDrive.getAimLockPID().calculate(currentAngle, targetAngle);
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, omegaRadiansPerSec);
    m_swerveDrive.runVelocity(chassisSpeeds);

    // Aim the wrist
    double distanceToTarget = TargetingUtil.getDistanceToSpeaker(m_swerveDrive.getPose());
    m_wrist.setAngleSetpoint(TargetingUtil.getWristAngle(distanceToTarget));

    // Wait for dwell after reaching setpoints
    if (m_swerveDrive.aimedAtSetpoint() && m_wrist.atSetpoint() && m_shooter.atSetpoint()) {
      if (m_dwellStart == 0) {
        m_dwellStart = Timer.getFPGATimestamp();
      } else if (Timer.getFPGATimestamp() - m_dwellStart > m_dwellDuration) {
        m_dwellFinished = true;
      }
    }

    // run feeder
    if (m_dwellFinished) {
      m_feeder.runAtVoltage(FeederConstants.kFeedVoltage);

      // Simulate a note being shot by un-breaking the beambreak after a delay
      if (Robot.isSimulation()) {
        m_feeder.setBeambreakUnbrokenAfterDelay();
      }
    }

    // timeout if we don't have a note
    if (!m_intake.hasNote() && !m_feeder.getBeambreakBroken()) {
      if (m_noNoteDwellStart == 0) {
        m_noNoteDwellStart = Timer.getFPGATimestamp();
      } else if (Timer.getFPGATimestamp() - m_noNoteDwellStart > m_noNoteDwellDuration) {
        m_hadNote = true; // setting hadNote to true will end the command
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

    // reset dwells
    m_dwellStart = 0;
    m_dwellFinished = false;
    m_hadNote = false;
    m_noNoteDwellStart = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_feeder.getBeambreakBroken() && m_hadNote;
  }
}
