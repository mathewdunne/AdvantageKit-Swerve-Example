// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.AimDriveMode;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ModuleLocation;
import frc.robot.commands.AimAtSpeakerCmd;
import frc.robot.commands.AmpCmd;
import frc.robot.commands.AutoSpeakerShotCmd;
import frc.robot.commands.AutoSubwooferShotCmd;
import frc.robot.commands.FeederAmpCmd;
import frc.robot.commands.FeederEjectCmd;
import frc.robot.commands.FeederShootCmd;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.SwerveDriveCmd;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmIOSparkMax;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederIOSim;
import frc.robot.subsystems.feeder.FeederIOSparkMax;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOSparkMax;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonReal;
import frc.robot.subsystems.vision.VisionIOPhotonSim;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristIO;
import frc.robot.subsystems.wrist.WristIOSim;
import frc.robot.subsystems.wrist.WristIOSparkMax;
import frc.robot.util.NoteVisualizer;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final SwerveDrive m_swerveDrive;
  private final Vision m_vision;
  private final Arm m_arm;
  private final Wrist m_wrist;
  private final Shooter m_shooter;
  private final Intake m_intake;
  private final Feeder m_feeder;

  // Controller
  private final CommandXboxController m_driverController =
      new CommandXboxController(ControllerConstants.kDriverControllerPort);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> m_autoChooser;

  // Commands
  private final SwerveDriveCmd m_driveCmd;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.kCurrentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        m_swerveDrive =
            new SwerveDrive(
                new GyroIONavX(),
                new ModuleIOSparkMax(ModuleLocation.FRONT_LEFT),
                new ModuleIOSparkMax(ModuleLocation.FRONT_RIGHT),
                new ModuleIOSparkMax(ModuleLocation.BACK_LEFT),
                new ModuleIOSparkMax(ModuleLocation.BACK_RIGHT));
        m_vision = new Vision(new VisionIOPhotonReal() {}, m_swerveDrive::addVisionMeasurement);
        m_arm = new Arm(new ArmIOSparkMax() {});
        m_wrist =
            new Wrist(
                new WristIOSparkMax(m_arm::getMechanismAngle) {},
                m_arm.getMechanismLigament(),
                m_arm::getTipPosition);
        m_shooter = new Shooter(new ShooterIOSparkMax());
        m_intake = new Intake(new IntakeIOSparkMax());
        m_feeder = new Feeder(new FeederIOSparkMax());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        m_swerveDrive =
            new SwerveDrive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        m_vision = new Vision(new VisionIOPhotonSim() {}, m_swerveDrive::addVisionMeasurement);
        m_vision.setSimTruePoseSupplier(m_swerveDrive::getSimTruePose);
        m_arm = new Arm(new ArmIOSim() {});
        m_wrist =
            new Wrist(
                new WristIOSim(m_arm::getMechanismAngle) {},
                m_arm.getMechanismLigament(),
                m_arm::getTipPosition);
        m_shooter = new Shooter(new ShooterIOSim());
        m_intake = new Intake(new IntakeIOSim());
        m_feeder = new Feeder(new FeederIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        m_swerveDrive =
            new SwerveDrive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        m_vision = new Vision(new VisionIO() {}, m_swerveDrive::addVisionMeasurement);
        m_arm = new Arm(new ArmIO() {});
        m_wrist = new Wrist(new WristIO() {}, m_arm.getMechanismLigament(), m_arm::getTipPosition);
        m_shooter = new Shooter(new ShooterIO() {});
        m_intake = new Intake(new IntakeIO() {});
        m_feeder = new Feeder(new FeederIO() {});
        break;
    }

    // Create master drive command
    m_driveCmd =
        new SwerveDriveCmd(
            m_swerveDrive,
            () -> -m_driverController.getLeftY(),
            () -> -m_driverController.getLeftX(),
            () -> -m_driverController.getRightX(),
            () -> m_vision.getClosestNote());

    // Set up auto routines
    NamedCommands.registerCommand(
        "shoot", new AutoSpeakerShotCmd(m_swerveDrive, m_wrist, m_shooter, m_feeder, m_intake));
    NamedCommands.registerCommand("subwooferShot", new AutoSubwooferShotCmd(m_feeder, m_wrist));
    NamedCommands.registerCommand(
        "eject",
        new FeederEjectCmd(m_feeder, m_wrist, m_arm, m_vision::manageNotesInSimulation)
            .withTimeout(1));

    m_autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    addSysIDRoutines(m_autoChooser);

    // Configure the button bindings
    configureButtonBindings();

    // Set suppliers for note visualizer
    if (Robot.isSimulation()) {
      NoteVisualizer.setRobotPoseSupplier(m_swerveDrive::getSimTruePose);
    } else {
      NoteVisualizer.setRobotPoseSupplier(m_swerveDrive::getPose);
    }
    NoteVisualizer.setWristPoseSupplier(m_wrist::getPose3d);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_swerveDrive.setDefaultCommand(m_driveCmd);

    // Create custom triggers based on the right trigger input and arm position
    Trigger rightTriggerArmUp =
        new Trigger(() -> m_driverController.getRightTriggerAxis() > 0.1 && !m_arm.isStowed());

    Trigger rightTriggerArmDown =
        new Trigger(() -> m_driverController.getRightTriggerAxis() > 0.1 && m_arm.isStowed());

    // Toggle field oriented / robot oriented drive
    m_driverController
        .leftStick()
        .onTrue(new InstantCommand(() -> m_driveCmd.toggleBaseDriveMode()));
    // Reset robot pose to 0, 0
    m_driverController
        .rightStick()
        .onTrue(new InstantCommand(() -> m_swerveDrive.resetOdometry()).ignoringDisable(true));

    // Arm up
    m_driverController
        .povUp()
        .whileTrue(
            new InstantCommand(() -> m_arm.runVolts(0.7 * RobotController.getBatteryVoltage())))
        .onFalse(new InstantCommand(() -> m_arm.stopAndHold()));
    // Arm down
    m_driverController
        .povDown()
        .whileTrue(
            new InstantCommand(() -> m_arm.runVolts(-0.7 * RobotController.getBatteryVoltage())))
        .onFalse(new InstantCommand(() -> m_arm.stopAndHold()));

    // Wrist up
    m_driverController
        .povRight()
        .whileTrue(
            new InstantCommand(() -> m_wrist.runVolts(-0.3 * RobotController.getBatteryVoltage())))
        .onFalse(new InstantCommand(() -> m_wrist.stopAndHold()));
    // Wrist down
    m_driverController
        .povLeft()
        .whileTrue(
            new InstantCommand(() -> m_wrist.runVolts(0.3 * RobotController.getBatteryVoltage())))
        .onFalse(new InstantCommand(() -> m_wrist.stopAndHold()));

    // Intake
    m_driverController
        .rightBumper()
        .whileTrue(
            new IntakeCmd(
                m_intake,
                m_feeder,
                m_wrist,
                m_arm,
                m_driverController,
                m_swerveDrive::getPose,
                m_vision::manageNotesInSimulation));

    // Track Note
    m_driverController
        .leftBumper()
        .whileTrue(
            new InstantCommand(
                () -> {
                  m_driveCmd.setAimDriveMode(AimDriveMode.TRACK_NOTE);
                  m_driveCmd.setRobotOriented();
                }))
        .onFalse(
            new InstantCommand(
                () -> {
                  m_driveCmd.setAimDriveMode(AimDriveMode.NONE);
                  m_driveCmd.setFieldOriented();
                }));

    // Aim for speaker shot
    m_driverController
        .leftTrigger(0.1)
        .whileTrue(
            new AimAtSpeakerCmd(
                m_shooter,
                m_feeder,
                m_wrist,
                m_arm,
                m_driverController,
                m_swerveDrive::getPose,
                m_driveCmd::setAimDriveMode));

    // Amp
    m_driverController
        .a()
        .onTrue(new AmpCmd(m_shooter, m_feeder, m_wrist, m_arm, m_driveCmd::setAimDriveMode));

    // Shoot
    rightTriggerArmDown.whileTrue(
        new FeederShootCmd(m_feeder, m_shooter, m_wrist, m_swerveDrive::aimedAtSetpoint));
    rightTriggerArmUp.whileTrue(new FeederAmpCmd(m_feeder, m_wrist, m_arm));

    // snap to amp
    m_driverController
        .start()
        .onTrue(new InstantCommand(() -> m_driveCmd.setAimDriveMode(AimDriveMode.FACE_AMP)));

    // snap to source
    m_driverController
        .b()
        .onTrue(new InstantCommand(() -> m_driveCmd.setAimDriveMode(AimDriveMode.FACE_SOURCE)));

    // Face forward
    m_driverController
        .y()
        .onTrue(new InstantCommand(() -> m_driveCmd.setAimDriveMode(AimDriveMode.FACE_FORWARD)));

    // Face backward
    m_driverController
        .x()
        .onTrue(new InstantCommand(() -> m_driveCmd.setAimDriveMode(AimDriveMode.FACE_BACKWARD)));

    // Eject
    m_driverController
        .back()
        .whileTrue(new FeederEjectCmd(m_feeder, m_wrist, m_arm, m_vision::manageNotesInSimulation));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.get();
  }

  // Add sysID routines to auto chooser
  public void addSysIDRoutines(LoggedDashboardChooser<Command> autoChooser) {
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        m_swerveDrive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        m_swerveDrive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)",
        m_swerveDrive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)",
        m_swerveDrive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Shooter SysId (Quasistatic Forward)",
        m_shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Shooter SysId (Quasistatic Reverse)",
        m_shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Shooter SysId (Dynamic Forward)", m_shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Shooter SysId (Dynamic Reverse)", m_shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Arm SysId (Quasistatic Forward)", m_arm.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Arm SysId (Quasistatic Reverse)", m_arm.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Arm SysId (Dynamic Forward)", m_arm.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Arm SysId (Dynamic Reverse)", m_arm.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Wrist SysId (Quasistatic Forward)",
        m_wrist.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Wrist SysId (Quasistatic Reverse)",
        m_wrist.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Wrist SysId (Dynamic Forward)", m_wrist.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Wrist SysId (Dynamic Reverse)", m_wrist.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  // Getters
  public Shooter getShooter() {
    return m_shooter;
  }

  public Intake getIntake() {
    return m_intake;
  }

  public Feeder getFeeder() {
    return m_feeder;
  }

  public Pose2d getRobotPose() {
    return m_swerveDrive.getPose();
  }

  public Vision getVision() {
    return m_vision;
  }

  public CommandXboxController getController() {
    return m_driverController;
  }

  public Wrist getWrist() {
    return m_wrist;
  }

  public Arm getArm() {
    return m_arm;
  }
}
