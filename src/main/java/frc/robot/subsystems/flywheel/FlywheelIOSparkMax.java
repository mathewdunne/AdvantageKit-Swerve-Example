// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.FlywheelConstants;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class FlywheelIOSparkMax implements FlywheelIO {
  private static final double m_gearRatio = FlywheelConstants.kGearRatio;

  private final CANSparkMax m_leader =
      new CANSparkMax(FlywheelConstants.kLeaderMotorPort, MotorType.kBrushless);
  private final CANSparkMax m_follower =
      new CANSparkMax(FlywheelConstants.kFollowerMotorPort, MotorType.kBrushless);
  private final RelativeEncoder m_encoder = m_leader.getEncoder();
  private final SparkPIDController m_pid = m_leader.getPIDController();

  public FlywheelIOSparkMax() {
    m_leader.restoreFactoryDefaults();
    m_follower.restoreFactoryDefaults();

    m_leader.setCANTimeout(250);
    m_follower.setCANTimeout(250);

    m_leader.setInverted(false);
    m_follower.follow(m_leader, false);

    m_leader.enableVoltageCompensation(12.0);
    m_leader.setSmartCurrentLimit(FlywheelConstants.kCurrentLimit);

    m_leader.burnFlash();
    m_follower.burnFlash();
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(m_encoder.getPosition() / m_gearRatio);
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(m_encoder.getVelocity() / m_gearRatio);
    inputs.appliedVolts = m_leader.getAppliedOutput() * m_leader.getBusVoltage();
    inputs.currentAmps = new double[] {m_leader.getOutputCurrent(), m_follower.getOutputCurrent()};
  }

  @Override
  public void setVoltage(double volts) {
    m_leader.setVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    m_pid.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec) * m_gearRatio,
        ControlType.kVelocity,
        0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    m_leader.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    m_pid.setP(kP, 0);
    m_pid.setI(kI, 0);
    m_pid.setD(kD, 0);
    m_pid.setFF(0, 0);
  }
}
