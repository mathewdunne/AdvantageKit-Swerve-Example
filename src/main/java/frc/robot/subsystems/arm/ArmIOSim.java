// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class ArmIOSim implements ArmIO {
  private SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          DCMotor.getFalcon500(1),
          ArmConstants.kGearRatio,
          ArmConstants.kMOIkgm2,
          ArmConstants.kLengthMeters,
          ArmConstants.kMinAngleRad,
          ArmConstants.kMaxAngleRad,
          true,
          ArmConstants.kMinAngleRad);

  private double m_armAppliedVolts = 0.0;

  public ArmIOSim() {
    System.out.println("[Init] CreatingArmIOSim");
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    armSim.update(Constants.kLoopPeriodSecs);

    inputs.absolutePositionRad = armSim.getAngleRads();
    inputs.internalPositionRad = armSim.getAngleRads();
    inputs.velocityRadPerSec = armSim.getVelocityRadPerSec();
    inputs.appliedVolts = m_armAppliedVolts;
    inputs.currentAmps = new double[] {armSim.getCurrentDrawAmps()};
    inputs.tempCelsius = new double[] {};
  }

  @Override
  public void setVoltage(double voltage) {
    m_armAppliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
    armSim.setInputVoltage(m_armAppliedVolts);
  }
}
