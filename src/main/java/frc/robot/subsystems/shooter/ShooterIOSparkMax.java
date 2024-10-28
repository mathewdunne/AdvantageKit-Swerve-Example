// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ShooterConstants;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class ShooterIOSparkMax implements ShooterIO {

  private final CANSparkMax m_motor1 =
      new CANSparkMax(ShooterConstants.kLeaderMotorPort, MotorType.kBrushless);
  private final CANSparkMax m_motor2 =
      new CANSparkMax(ShooterConstants.kFollowerMotorPort, MotorType.kBrushless);
  private final RelativeEncoder m_encoder1 = m_motor1.getEncoder();
  private final RelativeEncoder m_encoder2 = m_motor1.getEncoder();

  public ShooterIOSparkMax() {
    // m_motor1.restoreFactoryDefaults();
    // m_motor2.restoreFactoryDefaults();

    m_motor1.setCANTimeout(250);
    m_motor2.setCANTimeout(250);

    m_motor1.setInverted(false);
    m_motor2.setInverted(false);

    m_motor1.enableVoltageCompensation(12.0);
    m_motor1.setSmartCurrentLimit(ShooterConstants.kCurrentLimit);
    m_motor2.enableVoltageCompensation(12.0);
    m_motor2.setSmartCurrentLimit(ShooterConstants.kCurrentLimit);

    // m_motor1.burnFlash();
    // m_motor2.burnFlash();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.positionRad =
        new double[] {
          Units.rotationsToRadians(m_encoder1.getPosition()),
          Units.rotationsToRadians(m_encoder2.getPosition())
        };
    inputs.velocityRadPerSec =
        new double[] {
          Units.rotationsPerMinuteToRadiansPerSecond(m_encoder1.getVelocity()),
          Units.rotationsPerMinuteToRadiansPerSecond(m_encoder2.getVelocity())
        };
    inputs.appliedVolts =
        new double[] {
          m_motor1.getAppliedOutput() * m_motor1.getBusVoltage(),
          m_motor2.getAppliedOutput() * m_motor2.getBusVoltage()
        };
    inputs.currentAmps = new double[] {m_motor1.getOutputCurrent(), m_motor2.getOutputCurrent()};
    inputs.tempCelsius =
        new double[] {m_motor1.getMotorTemperature(), m_motor2.getMotorTemperature()};
  }

  @Override
  public void setVoltage(double volts) {
    m_motor1.setVoltage(volts);
    m_motor2.setVoltage(volts);
  }

  @Override
  public void setVoltage(double motor1Volts, double motor2Volts) {
    m_motor1.setVoltage(motor1Volts);
    m_motor2.setVoltage(motor2Volts);
  }

  @Override
  public void stop() {
    m_motor1.stopMotor();
    m_motor2.stopMotor();
  }
}
