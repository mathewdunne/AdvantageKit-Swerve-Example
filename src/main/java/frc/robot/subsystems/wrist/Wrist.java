// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.WristConstants;
import frc.robot.util.TunablePIDController;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  private final WristIO m_io;
  private final WristIOInputsAutoLogged m_inputs = new WristIOInputsAutoLogged();

  private boolean m_isPidEnabled;

  private MechanismLigament2d m_armMechanismLigament;
  private MechanismLigament2d m_shooterMechanismLigament;
  private MechanismLigament2d m_intakeMechanismLigament;
  private Supplier<Translation3d> m_mechanismRootSupplier;

  private final PIDController m_pidController;
  private final ArmFeedforward m_ffModel;
  private final SysIdRoutine m_sysId;

  public Wrist(
      WristIO io, MechanismLigament2d armLigament, Supplier<Translation3d> getWristRootSupplier) {
    System.out.println("[Init] Creating Wrist");
    m_io = io;
    m_armMechanismLigament = armLigament;
    m_mechanismRootSupplier = getWristRootSupplier;

    // 2 ligaments for the wrist since the pibot point is in the middle
    // just always 180 degrees apart
    m_shooterMechanismLigament =
        m_armMechanismLigament.append(
            new MechanismLigament2d(
                "WristShooterLigament",
                WristConstants.kLengthMeters,
                Units.radiansToDegrees(WristConstants.kStartAngleRad),
                4,
                new Color8Bit(255, 0, 0)));
    m_intakeMechanismLigament =
        m_armMechanismLigament.append(
            new MechanismLigament2d(
                "WristIntakeLigament",
                WristConstants.kLengthMeters,
                WristConstants.kStartAngleRad + 180,
                4,
                m_shooterMechanismLigament.getColor()));

    // ---------------------------------------------------------------------------------------------------------------
    // TUNE THESE VALUES
    m_ffModel = new ArmFeedforward(0.0, 3.1407, 0.79481, 0.037738);
    m_pidController =
        new TunablePIDController(10, 0, 0, WristConstants.kToleranceRad, "Wrist", true);
    // ---------------------------------------------------------------------------------------------------------------

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
      // real world position to compensate for arm movement
      pid = m_pidController.calculate(m_inputs.absolutePositionRad);
      // FF calculation needs to use the real world position.
      ff = m_ffModel.calculate(m_inputs.realWorldPositionRad, 0);
      m_io.setVoltage((pid + ff));
    }

    // Update mechanism2d
    // 0 degrees for ligament is straight in line with the parent ligament
    // Subtracting 180 degrees makes it move the right way
    m_shooterMechanismLigament.setAngle(Units.radiansToDegrees(m_inputs.absolutePositionRad) - 180);
    m_intakeMechanismLigament.setAngle(Units.radiansToDegrees(m_inputs.absolutePositionRad));

    // Log the wrist pose
    Logger.recordOutput("Mechanism3d/Wrist", getPose3d(m_inputs.realWorldPositionRad));
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
    m_io.setVoltage(0.0);
    m_pidController.setSetpoint(m_inputs.absolutePositionRad);
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
    return new Pose3d(m_mechanismRootSupplier.get(), new Rotation3d(0, -angleRad, 0));
  }
}
