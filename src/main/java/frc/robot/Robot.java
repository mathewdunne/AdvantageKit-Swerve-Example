// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.NoteVisualizer;
import frc.robot.util.TargetingUtil;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  // timer for simulating intake in auto
  private double m_noteIntakeStartTime = 0;
  private double m_nearNoteTimerDuration = 0.5;
  private int m_nearNoteIndex = -1;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.kCurrentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        // Logger.addDataReceiver(new WPILOGWriter());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // See http://bit.ly/3YIzFZ6 for more information on timestamps in AdvantageKit.
    // Logger.disableDeterministicTimestamps()

    // Start AdvantageKit logger
    Logger.start();

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Update NoteVisualizer
    NoteVisualizer.showHeldNotes();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // set beambreak broken if in simulation
    if (Constants.kCurrentMode == Constants.Mode.SIM) {
      m_robotContainer.getFeeder().setBeambreakBroken();
    }

    // schedule the autonomous command
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    // Reset NoteVisualizer
    NoteVisualizer.resetFieldNotes();
    NoteVisualizer.showFieldNotes();
    NoteVisualizer.setHasNote(true);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // spin shooter
    double rpm =
        TargetingUtil.getShooterRPM(
            TargetingUtil.getDistanceToSpeaker(m_robotContainer.getRobotPose()));
    m_robotContainer.getShooter().runAtVelocityRPM(rpm);

    // run intake
    if (!m_robotContainer.getFeeder().getBeambreakBroken()) {
      m_robotContainer.getIntake().runAtVoltage(IntakeConstants.kIntakeVoltage);
      m_robotContainer.getFeeder().runAtVoltage(FeederConstants.kIntakeVoltage);
    } else {
      // stop intake and feeder
      m_robotContainer.getIntake().stop();
      // only manually stop feeder if wrist is stowed so that it can run when shooting
      if (m_robotContainer.getWrist().isStowed()) {
        m_robotContainer.getFeeder().stop();
      }
    }

    // sim intake if the robot is near a note
    if (Constants.kCurrentMode == Constants.Mode.SIM
        && !m_robotContainer.getFeeder().getBeambreakBroken()) {
      Pose3d intakePose =
          new Pose3d(m_robotContainer.getRobotPose()).transformBy(IntakeConstants.kRobotToIntake);
      boolean nearNote = false;
      for (int i = 0; i < NoteVisualizer.getFieldNotes().size(); i++) {
        Translation2d notePose = NoteVisualizer.getFieldNotes().get(i);
        if (notePose != null
            && intakePose.getTranslation().toTranslation2d().getDistance(notePose) < 0.5) {
          nearNote = true;
          m_nearNoteIndex = i;
          break;
        }
      }
      Logger.recordOutput("Intake/SimNearNote", nearNote);

      if (nearNote) {
        if (m_noteIntakeStartTime == 0) {
          m_noteIntakeStartTime = Timer.getFPGATimestamp();
        } else if (Timer.getFPGATimestamp() - m_noteIntakeStartTime >= m_nearNoteTimerDuration) {
          // Simulate a note being intaked by breaking the beambreak after a delay
          m_robotContainer.getFeeder().setBeambreakBroken();

          // remove the note from the field
          NoteVisualizer.setHasNote(true);
          NoteVisualizer.takeFieldNote(m_nearNoteIndex);
          m_robotContainer.getVision().manageNotesInSimulation(m_nearNoteIndex);
          m_nearNoteIndex = -1;
        }
      } else {
        m_noteIntakeStartTime = 0;
      }
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // Reset NoteVisualizer
    NoteVisualizer.resetFieldNotes();
    NoteVisualizer.showFieldNotes();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
