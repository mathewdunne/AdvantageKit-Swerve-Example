// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.SetAimDriveModeCallback;

public class AmpCmd extends Command {

  private final Shooter m_shooter;
  private final Feeder m_feeder;
  private final Wrist m_wrist;
  private final Arm m_arm;
  private final SetAimDriveModeCallback m_setAimDriveModeCallback;

  /** Creates a new AmpCommand. */
  public AmpCmd(
      Shooter shooter,
      Feeder feeder,
      Wrist wrist,
      Arm arm,
      SetAimDriveModeCallback setAimDriveModeCallback) {
    m_shooter = shooter;
    m_feeder = feeder;
    m_wrist = wrist;
    m_arm = arm;
    m_setAimDriveModeCallback = setAimDriveModeCallback;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_wrist, m_arm, m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Aim the chassis
    m_setAimDriveModeCallback.execute(Constants.AimDriveMode.FACE_AMP);

    // Move the arm
    m_arm.setAngleSetpoint(Constants.ArmConstants.kAmpAngleRad);

    // Move the wrist
    m_wrist.setAngleSetpoint(Constants.WristConstants.kAmpAngleRad);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.runSeparateVoltages(
        Constants.ShooterConstants.kAmpVoltages[0], Constants.ShooterConstants.kAmpVoltages[1]);

    // Not 100% sure why this is here
    if (!m_feeder.getBeambreakBroken()) {
      m_feeder.runAtVoltage(FeederConstants.kIntakeVoltage);
    } else {
      m_feeder.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
