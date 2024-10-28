// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
  @AutoLog
  public static class FeederIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double tempCelsius = 0.0;
    public boolean beambreakBroken = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(FeederIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Stop in open loop. */
  public default void stop() {}

  /** Set the state of the beambreak to broken after a short delay */
  public default void setBeambreakBrokenAfterDelay() {
      throw new UnsupportedOperationException("setBeambreakBrokenAfterDelay() is not supported outside of simulation");
  }

  /** Set the state of the beambreak to unbroken after a short delay */
  public default void setBeambreakUnbrokenAfterDelay() {
      throw new UnsupportedOperationException("setBeambreakUnbrokenAfterDelay() is not supported outside of simulation");
  }
}
