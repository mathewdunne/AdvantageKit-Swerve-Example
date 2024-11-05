// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.TargetingUtil;
import java.util.function.Supplier;

public class PreSpinShooterCmd extends Command {

  private final Shooter m_shooter;
  private final Supplier<Pose2d> m_poseSupplier;

  /** Creates a new PreSpinShooter. */
  public PreSpinShooterCmd(Shooter shooter, Supplier<Pose2d> poseSupplier) {

    m_shooter = shooter;
    m_poseSupplier = poseSupplier;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rpm =
        TargetingUtil.getShooterRPM(TargetingUtil.getDistanceToSpeaker(m_poseSupplier.get()));
    m_shooter.runAtVelocityRPM(rpm);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
