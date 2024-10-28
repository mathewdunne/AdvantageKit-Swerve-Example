// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO m_io;
  private final IntakeIOInputsAutoLogged m_inputs = new IntakeIOInputsAutoLogged();

  private double voltage = 0.0;

  /** Creates a new Intake. */
  public Intake(IntakeIO io) {
    System.out.println("[Init] Creating Intake");
    m_io = io;
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("Intake", m_inputs);

    runVolts(voltage);

    Logger.recordOutput(
        "Intake/RPM", Units.radiansPerSecondToRotationsPerMinute(m_inputs.velocityRadPerSec));
  }

  /** Run open loop at the specified voltage. Needs to be called periodically */
  private void runVolts(double volts) {
    m_io.setVoltage(volts);
  }

  /** Set a voltage to run at constantly */
  public void runAtVoltage(double volts) {
    voltage = volts;
  }

  /** Stops the Intake. */
  public void stop() {
    voltage = 0.0;
    m_io.stop();
  }
}
