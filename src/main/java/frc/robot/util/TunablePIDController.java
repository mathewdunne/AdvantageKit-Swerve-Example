// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** A PID controller that can be tuned on SmartDashboard. */
public class TunablePIDController extends PIDController {

  private final String m_key;
  private final boolean m_tuneMode;

  public TunablePIDController(
      double kP, double kI, double kD, double toleranceRad, String key, boolean tuneMode) {
    super(kP, kI, kD);
    if (toleranceRad >= 0) {
      setTolerance(toleranceRad);
    }
    m_key = key + "/PID";
    m_tuneMode = tuneMode;

    if (m_tuneMode) {
      SmartDashboard.putData(m_key, this);
    }
  }

  @Override
  public double calculate(double measurement) {
    if (m_tuneMode) {
      setP(SmartDashboard.getNumber(m_key + "/p", getP()));
      setI(SmartDashboard.getNumber(m_key + "/i", getI()));
      setD(SmartDashboard.getNumber(m_key + "/d", getD()));
    }
    return super.calculate(measurement);
  }
}
