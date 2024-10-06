// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FlywheelIOSim implements FlywheelIO {
  private FlywheelSim m_sim = new FlywheelSim(DCMotor.getNEO(1), 1.5, 0.004);
  private PIDController m_pid = new PIDController(0.0, 0.0, 0.0);

  private boolean m_closedLoop = false;
  private double m_ffVolts = 0.0;
  private double m_appliedVolts = 0.0;

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    if (m_closedLoop) {
      m_appliedVolts =
          MathUtil.clamp(
              m_pid.calculate(m_sim.getAngularVelocityRadPerSec()) + m_ffVolts, -12.0, 12.0);
      m_sim.setInputVoltage(m_appliedVolts);
    }

    m_sim.update(0.02);

    inputs.positionRad = 0.0;
    inputs.velocityRadPerSec = m_sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = m_appliedVolts;
    inputs.currentAmps = new double[] {m_sim.getCurrentDrawAmps()};
  }

  @Override
  public void setVoltage(double volts) {
    m_closedLoop = false;
    m_appliedVolts = volts;
    m_sim.setInputVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    m_closedLoop = true;
    m_pid.setSetpoint(velocityRadPerSec);
    this.m_ffVolts = ffVolts;
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    m_pid.setPID(kP, kI, kD);
  }
}
