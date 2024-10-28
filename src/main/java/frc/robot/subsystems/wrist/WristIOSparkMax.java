// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.WristConstants;
import java.util.function.Supplier;

public class WristIOSparkMax implements WristIO {
  private final CANSparkMax m_motor =
      new CANSparkMax(WristConstants.kMotorPort, MotorType.kBrushless);
  private final SparkAbsoluteEncoder m_absoluteEncoder =
      m_motor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

  private Supplier<Double> m_armAngleRadSupplier;

  public WristIOSparkMax(Supplier<Double> armAngleRadSupplier) {
    System.out.println("[Init] CreatingWristIOSparkMax");
    m_armAngleRadSupplier = armAngleRadSupplier;

    // m_motor.restoreFactoryDefaults();

    m_motor.setCANTimeout(250);

    m_motor.setInverted(false);

    m_motor.enableVoltageCompensation(12.0);
    m_motor.setSmartCurrentLimit(WristConstants.kCurrentLimit);

    // m_motor.burnFlash();
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.absolutePositionRad = Units.rotationsToRadians(m_absoluteEncoder.getPosition());
    inputs.internalPositionRad = Units.rotationsToRadians(m_motor.getEncoder().getPosition());
    // Real world position is (arm angle - 180 + the wrist angle) but in radians
    inputs.realWorldPositionRad =
        m_armAngleRadSupplier.get() - Math.PI + inputs.absolutePositionRad;
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(m_motor.getEncoder().getVelocity());
    inputs.appliedVolts = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
    inputs.currentAmps = m_motor.getOutputCurrent();
    inputs.tempCelsius = m_motor.getMotorTemperature();
  }

  @Override
  public void setVoltage(double volts) {
    m_motor.setVoltage(volts);
  }
}
