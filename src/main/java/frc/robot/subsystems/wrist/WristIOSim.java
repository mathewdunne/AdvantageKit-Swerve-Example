// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.Constants.WristConstants;

public class WristIOSim implements WristIO {
  private SingleJointedArmSim m_wristSim =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          WristConstants.kGearRatio,
          WristConstants.kMOIkgm2,
          WristConstants.kLengthMeters,
          WristConstants.kMinAngleRad,
          WristConstants.kMaxAngleRad,
          true,
          WristConstants.kMinAngleRad);

  private double m_wristAppliedVolts = 0.0;

  public WristIOSim() {
    System.out.println("[Init] CreatingWristIOSim");
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    m_wristSim.update(Constants.kLoopPeriodSecs);

    inputs.absolutePositionRad = m_wristSim.getAngleRads();
    inputs.internalPositionRad = m_wristSim.getAngleRads();
    inputs.velocityRadPerSec = m_wristSim.getVelocityRadPerSec();
    inputs.appliedVolts = m_wristAppliedVolts;
    inputs.currentAmps = new double[] {m_wristSim.getCurrentDrawAmps()};
    inputs.tempCelsius = new double[] {};
  }

  @Override
  public void setVoltage(double voltage) {
    m_wristAppliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
    m_wristSim.setInputVoltage(m_wristAppliedVolts);
  }
}
