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

  private final CANSparkMax m_leader =
      new CANSparkMax(ShooterConstants.kLeaderMotorPort, MotorType.kBrushless);
  private final CANSparkMax m_follower =
      new CANSparkMax(ShooterConstants.kFollowerMotorPort, MotorType.kBrushless);
  private final RelativeEncoder m_encoder = m_leader.getEncoder();

  public ShooterIOSparkMax() {
    m_leader.restoreFactoryDefaults();
    m_follower.restoreFactoryDefaults();

    m_leader.setCANTimeout(250);
    m_follower.setCANTimeout(250);

    m_leader.setInverted(false);
    m_follower.follow(m_leader, false);

    m_leader.enableVoltageCompensation(12.0);
    m_leader.setSmartCurrentLimit(ShooterConstants.kCurrentLimit);

    m_leader.burnFlash();
    m_follower.burnFlash();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(m_encoder.getVelocity());
    inputs.appliedVolts = m_leader.getAppliedOutput() * m_leader.getBusVoltage();
    inputs.currentAmps = new double[] {m_leader.getOutputCurrent(), m_follower.getOutputCurrent()};
    inputs.tempCelsius =
        new double[] {m_leader.getMotorTemperature(), m_follower.getMotorTemperature()};
  }

  @Override
  public void setVoltage(double volts) {
    m_leader.setVoltage(volts);
  }

  @Override
  public void stop() {
    m_leader.stopMotor();
  }
}
