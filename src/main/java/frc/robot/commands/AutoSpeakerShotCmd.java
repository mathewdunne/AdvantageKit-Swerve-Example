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
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.NoteVisualizer;
import frc.robot.util.TargetingUtil;

public class AutoSpeakerShotCmd extends Command {

  private final SwerveDrive m_swerveDrive;
  private final Wrist m_wrist;
  private final Shooter m_shooter;
  private final Feeder m_feeder;

  double m_timerStart = 0;
  boolean m_timerFinished = false;
  double m_delay = 0.2;

  boolean m_hadNote = false;

  /** Creates a new AutoSpeakerShotCmd. */
  public AutoSpeakerShotCmd(SwerveDrive swerveDrive, Wrist wrist, Shooter shooter, Feeder feeder) {
    m_swerveDrive = swerveDrive;
    m_wrist = wrist;
    m_shooter = shooter;
    m_feeder = feeder;

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

    // Wait for timer after reaching setpoints
    if (m_swerveDrive.aimedAtSetpoint() && m_wrist.atSetpoint() && m_shooter.atSetpoint()) {
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
    m_hadNote = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_feeder.getBeambreakBroken() && m_hadNote;
  }
}
