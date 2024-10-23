// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants;
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
  private Supplier<Double> m_armAngleRadFunction;

  public WristIOSim(Supplier<Double> armAngleRadFunction) {
    System.out.println("[Init] CreatingWristIOSim");
    m_armAngleRadFunction = armAngleRadFunction;
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    m_wristSim.setArmAngle(Math.PI - m_armAngleRadFunction.get());
    m_wristSim.update(Constants.kLoopPeriodSecs);

    inputs.absolutePositionRad = m_wristSim.getAngleRads();
    inputs.internalPositionRad = m_wristSim.getAngleRads();
    inputs.velocityRadPerSec = m_wristSim.getVelocityRadPerSec();
    inputs.appliedVolts = m_wristAppliedVolts;
    inputs.currentAmps = new double[] {m_wristSim.getCurrentDrawAmps()};
    inputs.tempCelsius = new double[] {};

    // Real world position is (arm angle - 180 + the wrist angle) but in radians
    inputs.realWorldPositionRad =
        m_armAngleRadFunction.get() - Math.PI + inputs.absolutePositionRad;
  }

  @Override
  public void setVoltage(double voltage) {
    m_wristAppliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
    m_wristSim.setInputVoltage(m_wristAppliedVolts);
  }
}
