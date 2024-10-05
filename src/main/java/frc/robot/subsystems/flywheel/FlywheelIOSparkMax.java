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
  private static final double GEAR_RATIO = FlywheelConstants.kGearRatio;

  private final CANSparkMax leader =
      new CANSparkMax(FlywheelConstants.kLeaderMotorPort, MotorType.kBrushless);
  private final CANSparkMax follower =
      new CANSparkMax(FlywheelConstants.kFollowerMotorPort, MotorType.kBrushless);
  private final RelativeEncoder encoder = leader.getEncoder();
  private final SparkPIDController pid = leader.getPIDController();

  public FlywheelIOSparkMax() {
    leader.restoreFactoryDefaults();
    follower.restoreFactoryDefaults();

    leader.setCANTimeout(250);
    follower.setCANTimeout(250);

    leader.setInverted(false);
    follower.follow(leader, false);

    leader.enableVoltageCompensation(12.0);
    leader.setSmartCurrentLimit(FlywheelConstants.kCurrentLimit);

    leader.burnFlash();
    follower.burnFlash();
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(encoder.getPosition() / GEAR_RATIO);
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / GEAR_RATIO);
    inputs.appliedVolts = leader.getAppliedOutput() * leader.getBusVoltage();
    inputs.currentAmps = new double[] {leader.getOutputCurrent(), follower.getOutputCurrent()};
  }

  @Override
  public void setVoltage(double volts) {
    leader.setVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    pid.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec) * GEAR_RATIO,
        ControlType.kVelocity,
        0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    leader.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setP(kP, 0);
    pid.setI(kI, 0);
    pid.setD(kD, 0);
    pid.setFF(0, 0);
  }
}
