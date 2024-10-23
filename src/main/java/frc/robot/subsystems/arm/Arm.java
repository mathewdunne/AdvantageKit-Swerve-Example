// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private final ArmIO m_io;
  private final ArmIOInputsAutoLogged m_inputs = new ArmIOInputsAutoLogged();

  private boolean m_isPidEnabled;

  private final Translation2d m_rootPosition;
  private Mechanism2d m_mechanism;
  private MechanismRoot2d m_mechanismRoot;
  private MechanismLigament2d m_mechanismLigament;

  private final PIDController m_pidController;
  private final ArmFeedforward m_ffModel;
  private final SysIdRoutine m_sysId;

  public Arm(ArmIO io) {
    System.out.println("[Init] Creating Arm");
    m_io = io;

    // Create the mechanism for visualization
    // with and height are for the "canvas" that shows the mechanism
    // set to the side profile of the robot (max height (y) and width (x))
    // the bottom center point of the Mechanism2d canvas aligns with the robot's origin
    m_mechanism = new Mechanism2d(Units.inchesToMeters(38.5), Units.inchesToMeters(48));
    // Position of the root/pivot point of the arm within the Mechanism2d canvas (0, 0 being the
    // bottom left corner of the side profile of the robot)
    // measured from CAD, the -2 is due to the robot's origin being the center of the swerve, not
    // the center of the robot
    m_rootPosition = new Translation2d(Units.inchesToMeters(32.25 - 4), Units.inchesToMeters(11.7));
    m_mechanismRoot = m_mechanism.getRoot("ArmWrist", m_rootPosition.getX(), m_rootPosition.getY());
    m_mechanismLigament =
        m_mechanismRoot.append(
            new MechanismLigament2d(
                "ArmLigament",
                ArmConstants.kLengthMeters,
                Units.radiansToDegrees(ArmConstants.kStartAngleRad),
                4,
                new Color8Bit(0, 0, 255)));

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.kCurrentMode) {
      case REAL:
        m_ffModel = new ArmFeedforward(ArmConstants.kS, ArmConstants.kV, ArmConstants.kG);
        m_pidController = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
        m_pidController.setTolerance(ArmConstants.kToleranceRad);
        break;
      case REPLAY:
        m_ffModel = new ArmFeedforward(0.1, 0.05, 0.01);
        m_pidController = new PIDController(1.0, 0.0, 0.0);
        m_pidController.setTolerance(ArmConstants.kToleranceRad);
        break;
      case SIM:
        m_ffModel = new ArmFeedforward(0.1, 0.05, 0.01);
        m_pidController = new PIDController(10.0, 0.0, 0.0);
        m_pidController.setTolerance(ArmConstants.kToleranceRad);
        break;
      default:
        m_ffModel = new ArmFeedforward(0.0, 0.0, 0.0);
        m_pidController = new PIDController(0.0, 0.0, 0.0);
        break;
    }

    // Set initial setpoint
    m_pidController.setSetpoint(ArmConstants.kStartAngleRad);
    m_isPidEnabled = true;

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
    m_mechanismLigament.setAngle(180 - Units.radiansToDegrees(m_inputs.absolutePositionRad));

    // Log the arm pose
    Logger.recordOutput("Mechanism2d/ArmWrist", m_mechanism);
    Logger.recordOutput("Mechanism3d/Arm", getPose3d(m_inputs.absolutePositionRad));
    Logger.recordOutput("Arm/AngleSetpointRad", m_pidController.getSetpoint());
    Logger.recordOutput("Arm/ActualAngleRad", m_inputs.absolutePositionRad);
    Logger.recordOutput("Arm/PID", pid);
    Logger.recordOutput("Arm/FF", ff);
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    m_isPidEnabled = false;
    m_io.setVoltage(volts);
  }

  /** Set the setpoint for PID control. (in radians, with 0 being horizontal and up being PI/2) */
  public void setAngleSetpoint(double angleRad) {
    if (angleRad < ArmConstants.kMinAngleRad) {
      angleRad = ArmConstants.kMinAngleRad;
    } else if (angleRad > ArmConstants.kMaxAngleRad) {
      angleRad = ArmConstants.kMaxAngleRad;
    }
    m_pidController.setSetpoint(angleRad);
  }

  /** Stop and hold the arm at its current position */
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
    return new Pose3d(
        m_rootPosition.getX() - 0.49, -0.01, m_rootPosition.getY(), new Rotation3d(0, angleRad, 0));
    // these offsets are just due to the origin of the cad models being finicky
  }

  /** Returns the ligament of the arm's Mechanism2d. */
  public MechanismLigament2d getMechanismLigament() {
    return m_mechanismLigament;
  }
}
