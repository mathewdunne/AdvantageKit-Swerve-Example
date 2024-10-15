// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private final ArmIO m_io;
  private final ArmIOInputsAutoLogged m_inputs = new ArmIOInputsAutoLogged();
  private final PIDController m_pidController;
  private final SimpleMotorFeedforward m_ffModel;
  private final SysIdRoutine m_sysId;

  @AutoLogOutput(key = "Arm/Setpoint")
  private Double m_angleSetpoint = null; // Setpoint for closed loop control, null for open loop

  public Arm(ArmIO io) {
    m_io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.kCurrentMode) {
      case REAL:
        m_ffModel = new SimpleMotorFeedforward(ArmConstants.kS, ArmConstants.kV);
        m_pidController = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
        break;
      case REPLAY:
        m_ffModel = new SimpleMotorFeedforward(0.1, 0.05);
        m_pidController = new PIDController(1.0, 0.0, 0.0);
        break;
      case SIM:
        m_ffModel = new SimpleMotorFeedforward(0.1, 0.05);
        m_pidController = new PIDController(0.5, 0.0, 0.0);
        break;
      default:
        m_ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        m_pidController = new PIDController(0.0, 0.0, 0.0);
        break;
    }

    // Configure SysId
    m_sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Arm/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("Arm", m_inputs);

    // Run closed loop PID + FF control
    if (m_angleSetpoint != null) {
      m_pidController.setSetpoint(m_angleSetpoint);
      m_io.setVoltage(
          m_ffModel.calculate(m_inputs.armVelocityRadPerSec)
              + m_pidController.calculate(m_inputs.armPositionRad));
    }
    // Log the arm pose
    Logger.recordOutput("Mechanisms/Arm", getPose3d(m_inputs.absoluteArmPosition.getDegrees()));
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    m_io.setVoltage(volts);
  }

  /** Set the setpoint for PID control. (in radians, with 0 being straight up) */
  public void setAngleSetpoint(double angle) {
    m_angleSetpoint = angle;
  }

  /** Stop the arm. */
  public void stop() {
    m_angleSetpoint = null;
    m_io.setVoltage(0);
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysId.dynamic(direction);
  }

  /** Returns the 3D pose of the intake for visualization. */
  private Pose3d getPose3d(double angle) {
    return new Pose3d(0, 0.0, 0, new Rotation3d(0.0, -angle, 0.0));
  }
}
