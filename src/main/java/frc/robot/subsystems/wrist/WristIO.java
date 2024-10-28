// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    // Absolute is the through bore encoder on the wrist (this will be relative to the arm)
    public double absolutePositionRad = 0.0;
    // Internal is the relative encoder in the motor
    public double internalPositionRad = 0.0;
    // Real world is the position of the wrist taking the arm angle into account, with 0 being
    // horizontal
    public double realWorldPositionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(WristIOInputs inputs) {}

  /** Run the motor at the specified voltage */
  public default void setVoltage(double volts) {}
}
