// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
  private final FeederIO m_io;
  private final FeederIOInputsAutoLogged m_inputs = new FeederIOInputsAutoLogged();

  private double m_voltage = 0.0;

  /** Creates a new Feeder. */
  public Feeder(FeederIO io) {
    System.out.println("[Init] Creating Feeder");
    m_io = io;
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("Feeder", m_inputs);

    runVolts(m_voltage);

    Logger.recordOutput(
        "Feeder/RPM", Units.radiansPerSecondToRotationsPerMinute(m_inputs.velocityRadPerSec));
  }

  /** Run open loop at the specified voltage. Needs to be called periodically */
  private void runVolts(double volts) {
    m_io.setVoltage(volts);
  }

  /** Set a voltage to run at constantly */
  public void runAtVoltage(double volts) {
    m_voltage = volts;
  }

  /** Stops the Feeder. */
  public void stop() {
    m_voltage = 0.0;
    m_io.stop();

    // Cancel the beambreak broken after delay
    if (m_io instanceof FeederIOSim) {
      ((FeederIOSim) m_io).cancelSetBeambreak();
    }
  }

  /** Gets the state of the beambreak */
  public boolean getBeambreakBroken() {
    return m_inputs.beambreakBroken;
  }

  /** Sets the beambreak to be broken after a delay in simulation */
  public void setBeambreakBrokenAfterDelay() {
    if (m_io instanceof FeederIOSim) {
      ((FeederIOSim) m_io).setBeambreakBrokenAfterDelay();
    }
  }

  /** Sets the beambreak to be unbroken after a delay in simulation */
  public void setBeambreakUnbrokenAfterDelay() {
    if (m_io instanceof FeederIOSim) {
      ((FeederIOSim) m_io).setBeambreakUnbrokenAfterDelay();
    }
  }

  /** Sets the beambreak to be broken immediately in simulation */
  public void setBeambreakBroken() {
    if (m_io instanceof FeederIOSim) {
      ((FeederIOSim) m_io).setBeambreakBroken();
    }
  }
}
