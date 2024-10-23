// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.WristConstants;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  private final WristIO m_io;
  private final WristIOInputsAutoLogged m_inputs = new WristIOInputsAutoLogged();

  private boolean m_isPidEnabled;

  // private final Translation2d m_rootPosition;
  private MechanismLigament2d m_armMechanismLigament;
  private MechanismLigament2d m_wristMechanismLigament;

  private final PIDController m_pidController;
  private final ArmFeedforward m_ffModel;
  private final SysIdRoutine m_sysId;

  public Wrist(WristIO io, MechanismLigament2d armLigament) {
    System.out.println("[Init] Creating Wrist");
    m_io = io;
    m_armMechanismLigament = armLigament;

    // m_rootPosition = new Translation2d(Units.inchesToMeters(32.25 - 4),
    // Units.inchesToMeters(11.7));
    m_wristMechanismLigament =
        m_armMechanismLigament.append(
            new MechanismLigament2d(
                "WristLigament",
                WristConstants.kLengthMeters,
                Units.radiansToDegrees(WristConstants.kStartAngleRad),
                4,
                new Color8Bit(255, 0, 0)));

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.kCurrentMode) {
      case REAL:
        m_ffModel = new ArmFeedforward(WristConstants.kS, WristConstants.kV, WristConstants.kG);
        m_pidController =
            new PIDController(WristConstants.kP, WristConstants.kI, WristConstants.kD);
        m_pidController.setTolerance(WristConstants.kToleranceRad);
        break;
      case REPLAY:
        m_ffModel = new ArmFeedforward(0.1, 0.05, 0.01);
        m_pidController = new PIDController(1.0, 0.0, 0.0);
        m_pidController.setTolerance(WristConstants.kToleranceRad);
        break;
      case SIM:
        m_ffModel = new ArmFeedforward(0.1, 0.05, 0.01);
        m_pidController = new PIDController(10.0, 0.0, 0.0);
        m_pidController.setTolerance(WristConstants.kToleranceRad);
        break;
      default:
        m_ffModel = new ArmFeedforward(0.0, 0.0, 0.0);
        m_pidController = new PIDController(0.0, 0.0, 0.0);
        break;
    }

    // Set initial setpoint
    m_pidController.setSetpoint(WristConstants.kStartAngleRad);
    m_isPidEnabled = true;

    // Configure SysId
    m_sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Wrist/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("Wrist", m_inputs);

    // Run closed loop PID + FF control
    double pid = 0.0;
    double ff = 0.0;
    if (m_isPidEnabled) {
      ff = m_ffModel.calculate(m_inputs.absolutePositionRad, m_inputs.velocityRadPerSec);
      pid = m_pidController.calculate(m_inputs.absolutePositionRad);
      m_io.setVoltage((pid + ff));
    }

    // Update mechanism2d
    // Mechanism2d uses 0 degrees to the right and increases counterclockwise
    // Subtracting from 180 degrees mirrors the angle across the vertical axis
    m_wristMechanismLigament.setAngle(Units.radiansToDegrees(m_inputs.absolutePositionRad));

    // Log the wrist pose
    Logger.recordOutput("Mechanism3d/Wrist", getPose3d(m_inputs.absolutePositionRad));
    Logger.recordOutput("Wrist/AngleSetpointRad", m_pidController.getSetpoint());
    Logger.recordOutput("Wrist/ActualAngleRad", m_inputs.absolutePositionRad);
    Logger.recordOutput("Wrist/PID", pid);
    Logger.recordOutput("Wrist/FF", ff);
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    m_isPidEnabled = false;
    m_io.setVoltage(volts);
  }

  /** Set the setpoint for PID control. (in radians, with 0 being horizontal and up being PI/2) */
  public void setAngleSetpoint(double angleRad) {
    if (angleRad < WristConstants.kMinAngleRad) {
      angleRad = WristConstants.kMinAngleRad;
    } else if (angleRad > WristConstants.kMaxAngleRad) {
      angleRad = WristConstants.kMaxAngleRad;
    }
    m_pidController.setSetpoint(angleRad);
  }

  /** Stop and hold the wrist at its current position */
  public void stopAndHold() {
    m_pidController.setSetpoint(m_inputs.absolutePositionRad);
    m_pidController.reset();
    m_isPidEnabled = true;
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
  private Pose3d getPose3d(double angleRad) {
    // return new Pose3d(
    //     m_rootPosition.getX() - 0.49, -0.01, m_rootPosition.getY(), new Rotation3d(0, angleRad,
    // 0));
    return new Pose3d();
    // these offsets are just due to the origin of the cad models being finicky
  }
}
