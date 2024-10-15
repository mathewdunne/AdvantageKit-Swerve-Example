// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ArmIOSim implements ArmIO {
  private static final double LOOP_PERIOD_SECS = 0.02;
  private DCMotorSim m_motorSim = new DCMotorSim(DCMotor.getFalcon500(1), 75, 0.25);
  private double m_armAppliedVolts = 0.0;

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    m_motorSim.update(LOOP_PERIOD_SECS);
    inputs.armPositionRad = m_motorSim.getAngularPositionRad();
    inputs.absoluteArmPosition = new Rotation2d(inputs.armPositionRad);
    inputs.armVelocityRadPerSec = m_motorSim.getAngularVelocityRadPerSec();
    inputs.armAppliedVolts = m_armAppliedVolts;
    inputs.armCurrentAmps = new double[] {m_motorSim.getCurrentDrawAmps()};
  }

  @Override
  public void setVoltage(double volts) {
    m_armAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    m_motorSim.setInputVoltage(volts);
  }
}
