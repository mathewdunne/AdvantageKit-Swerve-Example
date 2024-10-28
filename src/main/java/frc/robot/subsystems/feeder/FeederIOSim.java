// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

public class FeederIOSim implements FeederIO {
  private FlywheelSim m_sim = new FlywheelSim(DCMotor.getNeo550(1), 1.5, 0.001);

  private double m_appliedVolts = 0.0;

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    m_sim.update(Constants.kLoopPeriodSecs);

    inputs.positionRad = 0.0;
    inputs.velocityRadPerSec = m_sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = m_appliedVolts;
    inputs.currentAmps = m_sim.getCurrentDrawAmps();
    inputs.tempCelsius = 0.0;
  }

  @Override
  public void setVoltage(double volts) {
    m_appliedVolts = volts;
    m_sim.setInputVoltage(volts);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }
}
