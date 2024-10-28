// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.TunablePIDController;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO m_io;
  private final ShooterIOInputsAutoLogged m_inputs = new ShooterIOInputsAutoLogged();
  private boolean m_isPidEnabled;
  private final PIDController m_pidController;
  private final SimpleMotorFeedforward m_ffModel;
  private final SysIdRoutine m_sysId;

  /** Creates a new Shooter. */
  public Shooter(ShooterIO io) {
    System.out.println("[Init] Creating Shooter");
    m_io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.kCurrentMode) {
      case REAL:
        m_ffModel = new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV);
        m_pidController =
            new TunablePIDController(
                ShooterConstants.kP,
                ShooterConstants.kI,
                ShooterConstants.kD,
                ShooterConstants.kToleranceRadPerSec,
                "Shooter",
                false);
        break;
      case REPLAY:
        m_ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        m_pidController =
            new TunablePIDController(
                1.0, 0.0, 0.0, ShooterConstants.kToleranceRadPerSec, "Shooter", false);
        break;
      case SIM:
        m_ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        m_pidController =
            new TunablePIDController(
                1.0, 0.0, 0.0, ShooterConstants.kToleranceRadPerSec, "Shooter", false);
        break;
      default:
        m_ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        m_pidController = new TunablePIDController(0, 0, 0, 0, "Shooter", false);
        break;
    }

    // PID off by default
    m_isPidEnabled = false;

    // Configure SysId
    m_sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Shooter/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("Shooter", m_inputs);

    // Run closed loop PID + FF control
    double pid = 0.0;
    double ff = 0.0;
    if (m_isPidEnabled) {
      pid = m_pidController.calculate(m_inputs.velocityRadPerSec);
      ff = m_ffModel.calculate(m_pidController.getSetpoint());
      m_io.setVoltage((pid + ff));
    }

    Logger.recordOutput(
        "Shooter/RPM", Units.radiansPerSecondToRotationsPerMinute(m_inputs.velocityRadPerSec));
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    m_isPidEnabled = false;
    m_io.setVoltage(volts);
  }

  /** Run closed loop at the specified velocity. */
  public void runVelocityRPM(double velocityRPM) {
    double velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    m_pidController.setSetpoint(velocityRadPerSec);
    m_isPidEnabled = true;

    // Log Shooter setpoint
    Logger.recordOutput("Shooter/SetpointRPM", velocityRPM);
  }

  /** Stops the Shooter. */
  public void stop() {
    m_isPidEnabled = false;
    m_io.stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysId.dynamic(direction);
  }
}
