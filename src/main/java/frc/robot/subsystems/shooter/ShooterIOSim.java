// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

public class ShooterIOSim implements ShooterIO {
  private FlywheelSim m_sim = new FlywheelSim(DCMotor.getNEO(1), 1.5, 0.01);

  private double m_appliedVolts = 0.0;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    m_sim.update(Constants.kLoopPeriodSecs);

    inputs.positionRad = new double[] {0.0, 0.0};
    inputs.velocityRadPerSec =
        new double[] {m_sim.getAngularVelocityRadPerSec(), m_sim.getAngularVelocityRadPerSec()};
    inputs.appliedVolts = new double[] {m_appliedVolts, m_appliedVolts};
    inputs.currentAmps = new double[] {m_sim.getCurrentDrawAmps(), m_sim.getCurrentDrawAmps()};
    inputs.tempCelsius = new double[] {};
  }

  @Override
  public void setVoltage(double volts) {
    m_appliedVolts = volts;
    m_sim.setInputVoltage(volts);
  }

  @Override
  public void setVoltage(double motor1Volts, double motor2volts) {
    setVoltage(motor1Volts);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }
}
