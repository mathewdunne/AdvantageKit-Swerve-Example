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

  private final PIDController m_motor1PID;
  private final PIDController m_motor2PID;
  private final SimpleMotorFeedforward m_motor1ff;
  private final SimpleMotorFeedforward m_motor2ff;
  private final SysIdRoutine m_sysId;

  /** Creates a new Shooter. */
  public Shooter(ShooterIO io) {
    System.out.println("[Init] Creating Shooter");
    m_io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.kCurrentMode) {
      case REAL:
        m_motor1ff = new SimpleMotorFeedforward(ShooterConstants.kS1, ShooterConstants.kV1);
        m_motor2ff = new SimpleMotorFeedforward(ShooterConstants.kS2, ShooterConstants.kV2);
        m_motor1PID =
            new TunablePIDController(
                ShooterConstants.kP1,
                ShooterConstants.kI1,
                ShooterConstants.kD1,
                ShooterConstants.kToleranceRadPerSec,
                "Shooter1",
                false);
        m_motor2PID =
            new TunablePIDController(
                ShooterConstants.kP2,
                ShooterConstants.kI2,
                ShooterConstants.kD2,
                ShooterConstants.kToleranceRadPerSec,
                "Shooter2",
                false);
        break;
      case REPLAY:
        m_motor1ff = m_motor2ff = new SimpleMotorFeedforward(0.0, 0.0);
        m_motor1PID =
            m_motor2PID =
                new TunablePIDController(
                    1.0, 0.0, 0.0, ShooterConstants.kToleranceRadPerSec, "Shooter", false);
        break;
      case SIM:
        m_motor1ff = m_motor2ff = new SimpleMotorFeedforward(0.0, 0.0);
        m_motor1PID =
            m_motor2PID =
                new TunablePIDController(
                    1.0, 0.0, 0.0, ShooterConstants.kToleranceRadPerSec, "Shooter", false);
        break;
      default:
        m_motor1ff = m_motor2ff = new SimpleMotorFeedforward(0.0, 0.0);
        m_motor1PID = m_motor2PID = new TunablePIDController(0, 0, 0, 0, "Shooter", false);
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
    double pid1 = 0.0;
    double pid2 = 0.0;
    double ff1 = 0.0;
    double ff2 = 0.0;
    if (m_isPidEnabled) {
      pid1 = m_motor1PID.calculate(m_inputs.velocityRadPerSec[0]);
      ff1 = m_motor1ff.calculate(m_motor1PID.getSetpoint());
      pid2 = m_motor2PID.calculate(m_inputs.velocityRadPerSec[1]);
      ff2 = m_motor2ff.calculate(m_motor2PID.getSetpoint());
      m_io.setVoltage(pid1 + ff1, pid2 + ff2);
    }

    Logger.recordOutput(
        "Shooter/RPM",
        new double[] {
          Units.radiansPerSecondToRotationsPerMinute(m_inputs.velocityRadPerSec[0]),
          Units.radiansPerSecondToRotationsPerMinute(m_inputs.velocityRadPerSec[1])
        });
    Logger.recordOutput("Shooter/PID", new double[] {pid1, pid2});
    Logger.recordOutput("Shooter/FF", new double[] {ff1, ff2});
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    m_isPidEnabled = false;
    m_io.setVoltage(volts);
  }

  /** Run closed loop at the specified velocity. */
  public void runVelocityRPM(double velocityRPM) {
    double velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    m_motor1PID.setSetpoint(velocityRadPerSec);
    m_motor2PID.setSetpoint(velocityRadPerSec);
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
