// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public double armPositionRad = 0.0;
    public Rotation2d absoluteArmPosition = new Rotation2d();
    public double armVelocityRadPerSec = 0.0;
    public double armAppliedVolts = 0.0;
    public double[] armCurrentAmps = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmIOInputs inputs) {}

  /** Run the motor at the specified voltage */
  public default void setVoltage(double volts) {}
}
