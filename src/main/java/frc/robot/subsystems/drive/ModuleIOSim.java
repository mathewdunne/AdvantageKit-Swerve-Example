// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

/**
 * Physics sim implementation of module IO.
 *
 * <p>Uses two Shooter sims for the drive and turn motors, with the absolute position initialized to
 * a random value. The Shooter sims are not physically accurate, but provide a decent approximation
 * for the behavior of the module.
 */
public class ModuleIOSim implements ModuleIO {

  private DCMotorSim m_driveSim = new DCMotorSim(DCMotor.getNEO(1), 6.75, 0.025);
  private DCMotorSim m_turnSim = new DCMotorSim(DCMotor.getNEO(1), 150.0 / 7.0, 0.004);

  private final Rotation2d m_turnAbsoluteInitPosition =
      new Rotation2d(Math.random() * 2.0 * Math.PI);
  private double m_driveAppliedVolts = 0.0;
  private double m_turnAppliedVolts = 0.0;

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    m_driveSim.update(Constants.kLoopPeriodSecs);
    m_turnSim.update(Constants.kLoopPeriodSecs);

    inputs.drivePositionRad = m_driveSim.getAngularPositionRad();
    inputs.driveVelocityRadPerSec = m_driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = m_driveAppliedVolts;
    inputs.driveCurrentAmps = new double[] {Math.abs(m_driveSim.getCurrentDrawAmps())};

    inputs.turnAbsolutePosition =
        new Rotation2d(m_turnSim.getAngularPositionRad()).plus(m_turnAbsoluteInitPosition);
    inputs.turnPosition = new Rotation2d(m_turnSim.getAngularPositionRad());
    inputs.turnVelocityRadPerSec = m_turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = m_turnAppliedVolts;
    inputs.turnCurrentAmps = new double[] {Math.abs(m_turnSim.getCurrentDrawAmps())};
  }

  @Override
  public void setDriveVoltage(double volts) {
    m_driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    m_driveSim.setInputVoltage(m_driveAppliedVolts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    m_turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    m_turnSim.setInputVoltage(m_turnAppliedVolts);
  }
}
