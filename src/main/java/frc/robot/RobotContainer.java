// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.ModuleLocation;
import frc.robot.commands.DriveCommands.MasterDriveCmd;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOSparkMax;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonSim;
import java.util.Random;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final SwerveSubsystem m_swerveDrive;
  private final Flywheel m_flywheel;
  private final Vision m_vision;
  private final Arm m_arm;

  // Controller
  private final CommandXboxController m_driverController =
      new CommandXboxController(ControllerConstants.kDriverControllerPort);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> m_autoChooser;
  private final LoggedDashboardNumber m_flywheelSpeedInput =
      new LoggedDashboardNumber("Flywheel Speed", FlywheelConstants.defaultSpeed);

  // Commands
  private final MasterDriveCmd m_masterDriveCmd;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.kCurrentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        m_swerveDrive =
            new SwerveSubsystem(
                new GyroIONavX(),
                new ModuleIOSparkMax(ModuleLocation.FRONT_LEFT),
                new ModuleIOSparkMax(ModuleLocation.FRONT_RIGHT),
                new ModuleIOSparkMax(ModuleLocation.BACK_LEFT),
                new ModuleIOSparkMax(ModuleLocation.BACK_RIGHT));
        m_flywheel = new Flywheel(new FlywheelIOSparkMax());
        m_vision = null; // TO DO
        m_arm = null; // TO DO
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        m_swerveDrive =
            new SwerveSubsystem(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        m_flywheel = new Flywheel(new FlywheelIOSim());
        m_vision =
            new Vision(
                new VisionIOPhotonSim() {},
                m_swerveDrive::addVisionMeasurement,
                m_swerveDrive::getSimTruePose);
        m_arm = new Arm(new ArmIOSim() {});
        break;

      default:
        // Replayed robot, disable IO implementations
        m_swerveDrive =
            new SwerveSubsystem(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        m_flywheel = new Flywheel(new FlywheelIO() {});
        m_vision = null; // TO DO
        m_arm = null; // TO DO
        break;
    }

    // Set up auto routines
    NamedCommands.registerCommand(
        "Run Flywheel",
        Commands.startEnd(
                () -> m_flywheel.runVelocity(m_flywheelSpeedInput.get()),
                m_flywheel::stop,
                m_flywheel)
            .withTimeout(5.0));
    m_autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    addSysIDRoutines(m_autoChooser);

    // Create master drive command
    m_masterDriveCmd =
        new MasterDriveCmd(
            m_swerveDrive,
            () -> -m_driverController.getLeftY(),
            () -> -m_driverController.getLeftX(),
            () -> -m_driverController.getRightX());

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_swerveDrive.setDefaultCommand(m_masterDriveCmd);

    m_driverController
        .leftStick()
        .onTrue(new InstantCommand(() -> m_masterDriveCmd.toggleBaseDriveMode()));
    m_driverController
        .rightStick()
        .onTrue(new InstantCommand(() -> m_swerveDrive.resetOdometry()).ignoringDisable(true));

    // Apply a random offset to pose estimator to test vision correction
    m_driverController
        .y()
        .onTrue(
            new InstantCommand(
                () -> {
                  Random rand = new Random();
                  Transform2d randomOffset =
                      new Transform2d(
                          new Translation2d(rand.nextDouble() * 4 - 2, rand.nextDouble() * 4 - 2),
                          new Rotation2d(rand.nextDouble() * 2 * Math.PI));
                  m_swerveDrive.setPose(m_swerveDrive.getPose().plus(randomOffset));
                }));

    m_driverController
        .rightTrigger(0.05)
        .whileTrue(
            new RepeatCommand(
                new InstantCommand(
                    () -> m_arm.runVolts(m_driverController.getRightTriggerAxis() * 12.0))))
        .onFalse(new InstantCommand(() -> m_arm.stop()));
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
        "Flywheel SysId (Quasistatic Forward)",
        m_flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Flywheel SysId (Quasistatic Reverse)",
        m_flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Flywheel SysId (Dynamic Forward)",
        m_flywheel.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Flywheel SysId (Dynamic Reverse)",
        m_flywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Arm SysId (Quasistatic Forward)", m_arm.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Arm SysId (Quasistatic Reverse)", m_arm.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Arm SysId (Dynamic Forward)", m_arm.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Arm SysId (Dynamic Reverse)", m_arm.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }
}
