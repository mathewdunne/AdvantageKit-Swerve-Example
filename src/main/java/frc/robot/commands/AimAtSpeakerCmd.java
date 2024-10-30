// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.SetAimDriveModeCallback;
import frc.robot.util.TargetingUtil;
import java.util.function.Supplier;

public class AimAtSpeakerCmd extends Command {

  private final Shooter m_shooter;
  private final Feeder m_feeder;
  private final Wrist m_wrist;
  private final Arm m_arm;
  private final CommandGenericHID m_controller;
  private final Supplier<Pose2d> m_robotPoseSupplier;
  private final SetAimDriveModeCallback m_setAimDriveModeCallback;

  /** Creates a new AimAtSpeakerCmd. */
  public AimAtSpeakerCmd(
      Shooter shooter,
      Feeder feeder,
      Wrist wrist,
      Arm arm,
      CommandGenericHID controller,
      Supplier<Pose2d> robotPoseSupplier,
      SetAimDriveModeCallback setAimDriveModeCallback) {

    m_shooter = shooter;
    m_feeder = feeder;
    m_wrist = wrist;
    m_arm = arm;
    m_controller = controller;
    m_robotPoseSupplier = robotPoseSupplier;
    m_setAimDriveModeCallback = setAimDriveModeCallback;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_wrist, m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Aim the chassis
    m_setAimDriveModeCallback.execute(Constants.AimDriveMode.AIM_SPEAKER);

    // Aim the wrist
    double distanceToTarget = TargetingUtil.getDistanceToSpeaker(m_robotPoseSupplier.get());
    m_wrist.setAngleSetpoint(TargetingUtil.getWristAngle(distanceToTarget));

    // Spin up the shooter
    m_shooter.runAtVelocityRPM(TargetingUtil.getShooterRPM(distanceToTarget));

    // Rumble the controller if ready to shoot
    if (m_shooter.atSetpoint()) {
      m_controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1);
    } else {
      m_controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_setAimDriveModeCallback.execute(Constants.AimDriveMode.NONE);
    m_controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
    m_shooter.stop();
    new StowArmWristCmd(m_arm, m_wrist).schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_feeder.getBeambreakBroken() || !m_arm.isStowed();
  }
}
