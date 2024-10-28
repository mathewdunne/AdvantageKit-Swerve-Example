// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.util.MovingFrameSingleJointedArmSim;
import java.util.function.Supplier;

public class WristIOSim implements WristIO {
  private MovingFrameSingleJointedArmSim m_wristSim =
      new MovingFrameSingleJointedArmSim(
          DCMotor.getNEO(1),
          WristConstants.kGearRatio,
          WristConstants.kMOIkgm2,
          WristConstants.kLengthMeters,
          WristConstants.kMinAngleRad,
          WristConstants.kMaxAngleRad,
          true,
          WristConstants.kMinAngleRad,
          ArmConstants.kStartAngleRad);

  private double m_wristAppliedVolts = 0.0;
  private Supplier<Double> m_armAngleRadSupplier;

  // private double lastVelocity = 0.0;
  // private long lastVelocityTime = Logger.getRealTimestamp();

  public WristIOSim(Supplier<Double> armAngleRadSupplier) {
    System.out.println("[Init] CreatingWristIOSim");
    m_armAngleRadSupplier = armAngleRadSupplier;
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    m_wristSim.setArmAngle(Math.PI - m_armAngleRadSupplier.get());
    m_wristSim.update(Constants.kLoopPeriodSecs);

    inputs.absolutePositionRad = m_wristSim.getAngleRads();
    inputs.internalPositionRad = m_wristSim.getAngleRads();
    inputs.velocityRadPerSec = m_wristSim.getVelocityRadPerSec();
    inputs.appliedVolts = m_wristAppliedVolts;
    inputs.currentAmps = m_wristSim.getCurrentDrawAmps();
    inputs.tempCelsius = 0.0;

    // Real world position is (arm angle - 180 + the wrist angle) but in radians
    inputs.realWorldPositionRad =
        m_armAngleRadSupplier.get() - Math.PI + inputs.absolutePositionRad;

    // long currentTime = Logger.getRealTimestamp();
    // Logger.recordOutput(
    //     "Wrist/Accel",
    //     (inputs.velocityRadPerSec - lastVelocity)
    //         / (currentTime - lastVelocityTime)
    //         * Math.pow(10, 6));
    // lastVelocity = inputs.velocityRadPerSec;
    // lastVelocityTime = currentTime;
  }

  @Override
  public void setVoltage(double voltage) {
    m_wristAppliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
    m_wristSim.setInputVoltage(m_wristAppliedVolts);
  }
}
