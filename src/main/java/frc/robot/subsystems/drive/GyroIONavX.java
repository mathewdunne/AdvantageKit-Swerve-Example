// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;

/** IO implementation for Pigeon2 */
public class GyroIONavX implements GyroIO {
  private final AHRS m_navx;

  public GyroIONavX() {
    m_navx = new AHRS(SPI.Port.kMXP);
    m_navx.reset();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = m_navx.isConnected();
    inputs.yawPosition = Rotation2d.fromDegrees(m_navx.getYaw());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(m_navx.getRate());
  }
}
