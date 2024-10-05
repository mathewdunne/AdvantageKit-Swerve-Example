// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.ModuleLocation;
import frc.robot.Constants.SwerveModuleConstants;

/**
 * Module IO implementation for SparkMax drive motor controller, SparkMax turn motor controller (NEO
 * or NEO 550), and analog absolute encoder connected to the RIO
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using a CANcoder, copy from "ModuleIOTalonFX")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOSparkMax implements ModuleIO {
  private final CANSparkMax driveSparkMax;
  private final CANSparkMax turnSparkMax;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnRelativeEncoder;
  private final AnalogInput turnAbsoluteEncoder;

  private final boolean isTurnMotorInverted = true;
  private final Rotation2d absoluteEncoderOffset;

  public ModuleIOSparkMax(ModuleLocation location) {
    switch (location) {
      case FRONT_LEFT:
        driveSparkMax =
            new CANSparkMax(SwerveModuleConstants.kFrontLeftDriveMotorPort, MotorType.kBrushless);
        turnSparkMax =
            new CANSparkMax(SwerveModuleConstants.kFrontLeftTurningMotorPort, MotorType.kBrushless);
        turnAbsoluteEncoder = new AnalogInput(SwerveModuleConstants.kFrontLeftEncoderPort);
        absoluteEncoderOffset = new Rotation2d(SwerveModuleConstants.kFrontLeftEncoderOffset);
        break;
      case FRONT_RIGHT:
        driveSparkMax =
            new CANSparkMax(SwerveModuleConstants.kFrontRightDriveMotorPort, MotorType.kBrushless);
        turnSparkMax =
            new CANSparkMax(
                SwerveModuleConstants.kFrontRightTurningMotorPort, MotorType.kBrushless);
        turnAbsoluteEncoder = new AnalogInput(SwerveModuleConstants.kFrontRightEncoderPort);
        absoluteEncoderOffset = new Rotation2d(SwerveModuleConstants.kFrontRightEncoderOffset);
        break;
      case BACK_LEFT:
        driveSparkMax =
            new CANSparkMax(SwerveModuleConstants.kBackLeftDriveMotorPort, MotorType.kBrushless);
        turnSparkMax =
            new CANSparkMax(SwerveModuleConstants.kBackLeftTurningMotorPort, MotorType.kBrushless);
        turnAbsoluteEncoder = new AnalogInput(SwerveModuleConstants.kBackLeftEncoderPort);
        absoluteEncoderOffset = new Rotation2d(SwerveModuleConstants.kBackLeftEncoderOffset);
        break;
      case BACK_RIGHT:
        driveSparkMax =
            new CANSparkMax(SwerveModuleConstants.kBackRightDriveMotorPort, MotorType.kBrushless);
        turnSparkMax =
            new CANSparkMax(SwerveModuleConstants.kBackRightTurningMotorPort, MotorType.kBrushless);
        turnAbsoluteEncoder = new AnalogInput(SwerveModuleConstants.kBackRightEncoderPort);
        absoluteEncoderOffset = new Rotation2d(SwerveModuleConstants.kBackRightEncoderOffset);
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    driveSparkMax.restoreFactoryDefaults();
    turnSparkMax.restoreFactoryDefaults();

    driveSparkMax.setCANTimeout(250);
    turnSparkMax.setCANTimeout(250);

    driveEncoder = driveSparkMax.getEncoder();
    turnRelativeEncoder = turnSparkMax.getEncoder();

    turnSparkMax.setInverted(isTurnMotorInverted);
    driveSparkMax.setSmartCurrentLimit(SwerveModuleConstants.kDriveCurrentLimit);
    turnSparkMax.setSmartCurrentLimit(SwerveModuleConstants.kTurnCurrentLimit);
    driveSparkMax.enableVoltageCompensation(12.0);
    turnSparkMax.enableVoltageCompensation(12.0);

    driveEncoder.setPosition(0.0);
    driveEncoder.setMeasurementPeriod(10);
    driveEncoder.setAverageDepth(2);

    turnRelativeEncoder.setPosition(0.0);
    turnRelativeEncoder.setMeasurementPeriod(10);
    turnRelativeEncoder.setAverageDepth(2);

    driveSparkMax.setCANTimeout(0);
    turnSparkMax.setCANTimeout(0);

    driveSparkMax.burnFlash();
    turnSparkMax.burnFlash();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad =
        Units.rotationsToRadians(driveEncoder.getPosition())
            / SwerveModuleConstants.kDriveGearRatio;
    inputs.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity())
            / SwerveModuleConstants.kDriveGearRatio;
    inputs.driveAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();
    inputs.driveCurrentAmps = new double[] {driveSparkMax.getOutputCurrent()};

    inputs.turnAbsolutePosition =
        new Rotation2d(
                turnAbsoluteEncoder.getVoltage() / RobotController.getVoltage5V() * 2.0 * Math.PI)
            .minus(absoluteEncoderOffset);
    inputs.turnPosition =
        Rotation2d.fromRotations(
                turnRelativeEncoder.getPosition() / SwerveModuleConstants.kTurnGearRatio)
            .plus(inputs.turnAbsolutePosition);
    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity())
            / SwerveModuleConstants.kTurnGearRatio;
    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = new double[] {turnSparkMax.getOutputCurrent()};
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveSparkMax.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnSparkMax.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    driveSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
