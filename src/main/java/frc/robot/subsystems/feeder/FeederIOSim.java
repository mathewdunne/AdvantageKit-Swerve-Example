// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import java.util.Random;

public class FeederIOSim implements FeederIO {
  private FlywheelSim m_sim = new FlywheelSim(DCMotor.getNeo550(1), 1.5, 0.001);
  private double m_appliedVolts = 0.0;

  private boolean m_beambreakBroken = false;
  private double m_timeToUpdateBeambreak = 0.0;
  private boolean m_updateBeambreakFlag = false;
  private boolean m_beambreakNextState = false;

  private Random m_rng = new Random();

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    m_sim.update(Constants.kLoopPeriodSecs);

    inputs.positionRad = 0.0;
    inputs.velocityRadPerSec = m_sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = m_appliedVolts;
    inputs.currentAmps = m_sim.getCurrentDrawAmps();
    inputs.tempCelsius = 0.0;

    // Update the beambreak state if one of the update after delays was called
    if (m_updateBeambreakFlag && Timer.getFPGATimestamp() > m_timeToUpdateBeambreak) {
      m_beambreakBroken = m_beambreakNextState;
      m_updateBeambreakFlag = false;
    }
    inputs.beambreakBroken = m_beambreakBroken;
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

  /** Set the state of the beambreak to broken after a short delay */
  public void setBeambreakBrokenAfterDelay() {
    if (m_beambreakBroken || m_updateBeambreakFlag) {
      return;
    }
    m_timeToUpdateBeambreak = Timer.getFPGATimestamp() + getRandomDouble(0.5, 1.1);
    m_updateBeambreakFlag = true;
    m_beambreakNextState = true;
  }

  /** Set the state of the beambreak to unbroken after a short delay */
  public void setBeambreakUnbrokenAfterDelay() {
    if (!m_beambreakBroken || m_updateBeambreakFlag) {
      return;
    }
    m_timeToUpdateBeambreak = Timer.getFPGATimestamp() + getRandomDouble(0.1, 0.4);
    m_updateBeambreakFlag = true;
    m_beambreakNextState = false;
  }

  /** Set the state of the beambreak to broken immediately */
  public void setBeambreakBroken() {
    m_beambreakBroken = true;
  }

  public void cancelSetBeambreak() {
    m_updateBeambreakFlag = false;
  }

  private double getRandomDouble(double min, double max) {
    return min + (max - min) * m_rng.nextDouble();
  }
}
