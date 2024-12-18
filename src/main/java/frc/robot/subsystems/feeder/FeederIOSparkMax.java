// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.FeederConstants;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class FeederIOSparkMax implements FeederIO {

  private final CANSparkMax m_motor =
      new CANSparkMax(FeederConstants.kMotorPort, MotorType.kBrushless);

  private DigitalInput m_beambreak = new DigitalInput(FeederConstants.kBeambreakPort);

  public FeederIOSparkMax() {
    // m_motor.restoreFactoryDefaults();

    m_motor.setCANTimeout(250);

    m_motor.setInverted(false);

    m_motor.enableVoltageCompensation(12.0);
    m_motor.setSmartCurrentLimit(FeederConstants.kCurrentLimit);

    // m_motor.burnFlash();
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(m_motor.getEncoder().getPosition());
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(m_motor.getEncoder().getVelocity());
    inputs.appliedVolts = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
    inputs.currentAmps = m_motor.getOutputCurrent();
    inputs.tempCelsius = m_motor.getMotorTemperature();
    inputs.beambreakBroken = m_beambreak.get();
  }

  @Override
  public void setVoltage(double volts) {
    m_motor.setVoltage(volts);
  }

  @Override
  public void stop() {
    m_motor.stopMotor();
  }
}
