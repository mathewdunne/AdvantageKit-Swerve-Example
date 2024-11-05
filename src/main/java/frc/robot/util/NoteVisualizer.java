// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Set;
import java.util.function.Supplier;
import java.util.stream.Stream;
import org.littletonrobotics.junction.Logger;

public class NoteVisualizer {
  private static final double shotSpeed = 15.0; // Meters per sec
  private static final double ejectSpeed = 1.5; // Meters per sec
  private static Supplier<Pose2d> robotPoseSupplier = Pose2d::new;
  private static Supplier<Pose3d> wristPoseSupplier = Pose3d::new;
  private static final List<Translation2d> fieldNotes = new ArrayList<>();
  private static boolean hasNote = false;

  private static final Translation3d blueSpeaker = new Translation3d(0.225, 5.55, 2.1);
  private static final Translation3d redSpeaker = new Translation3d(16.317, 5.55, 2.1);

  // Setters for static fields
  public static void setRobotPoseSupplier(Supplier<Pose2d> robotPoseSupplier) {
    NoteVisualizer.robotPoseSupplier = robotPoseSupplier;
  }

  public static void setWristPoseSupplier(Supplier<Pose3d> wristPoseSupplier) {
    NoteVisualizer.wristPoseSupplier = wristPoseSupplier;
  }

  public static void setHasNote(boolean hasNote) {
    NoteVisualizer.hasNote = hasNote;
  }

  // Getters for static fields
  public static boolean getHasNote() {
    return hasNote;
  }

  public static List<Translation2d> getFieldNotes() {
    // return all notes
    // result needs to be checked for nulls
    return fieldNotes;
  }

  /** Show all staged notes for alliance */
  public static void showFieldNotes() {
    if (fieldNotes.isEmpty()) {
      Logger.recordOutput("NoteVisualizer/FieldNotes", new Pose3d[] {});
    }
    // Show auto notes
    Stream<Translation2d> presentNotes = fieldNotes.stream().filter(Objects::nonNull);
    Logger.recordOutput(
        "NoteVisualizer/FieldNotes",
        presentNotes
            .map(
                translation ->
                    new Pose3d(
                        translation.getX(),
                        translation.getY(),
                        Units.inchesToMeters(1.0),
                        new Rotation3d()))
            .toArray(Pose3d[]::new));
  }

  public static void clearFieldNotes() {
    fieldNotes.clear();
  }

  /** Add all notes to be shown at the beginning of auto */
  public static void resetFieldNotes() {
    clearFieldNotes();
    List<Pose3d> notePoses = NoteModel.getNotePositions();
    for (int i = 0; i < notePoses.size(); i++) {
      fieldNotes.add(i, notePoses.get(i).toPose2d().getTranslation());
    }
    Logger.recordOutput("NoteVisualizer/ShotNotes", new Pose3d[] {});
  }

  /**
   * Take note from field notes
   *
   * @param note Index of note
   */
  public static void takeFieldNote(int note) {
    fieldNotes.set(note, null);
    // refresh the notes shown on the field
    NoteVisualizer.showFieldNotes();
  }

  /** Shows the currently held note if there is one */
  public static void showHeldNotes() {
    if (hasNote) {
      Logger.recordOutput("NoteVisualizer/HeldNotes", new Pose3d[] {getIndexerPose3d()});
    } else {
      Logger.recordOutput("NoteVisualizer/HeldNotes", new Pose3d[] {});
    }
  }

  /** Shoots note from middle of arm to speaker */
  public static Command shoot() {
    return new ScheduleCommand( // Branch off and exit immediately
        Commands.defer(
                () -> {
                  hasNote = false;
                  boolean isRed =
                      DriverStation.getAlliance().isPresent()
                          && DriverStation.getAlliance().get() == Alliance.Red;
                  final Pose3d startPose = getIndexerPose3d();
                  final Pose3d endPose =
                      new Pose3d(isRed ? redSpeaker : blueSpeaker, startPose.getRotation());

                  final double duration =
                      startPose.getTranslation().getDistance(endPose.getTranslation()) / shotSpeed;
                  final Timer timer = new Timer();
                  timer.start();
                  return Commands.run(
                          () ->
                              Logger.recordOutput(
                                  "NoteVisualizer/ShotNotes",
                                  new Pose3d[] {
                                    startPose.interpolate(endPose, timer.get() / duration)
                                  }))
                      .until(() -> timer.hasElapsed(duration))
                      .finallyDo(
                          () -> Logger.recordOutput("NoteVisualizer/ShotNotes", new Pose3d[] {}));
                },
                Set.of())
            .ignoringDisable(true));
  }

  /** Shoots note from middle of arm to just in front (roughly where amp should be) */
  public static Command amp() {
    return new ScheduleCommand( // Branch off and exit immediately
        Commands.defer(
                () -> {
                  hasNote = false;
                  final Pose3d startPose = getIndexerPose3d();
                  final Pose3d endPose =
                      startPose.transformBy(new Transform3d(0.5, 0, 0, new Rotation3d()));

                  final double duration =
                      startPose.getTranslation().getDistance(endPose.getTranslation()) / ejectSpeed;
                  final Timer timer = new Timer();
                  timer.start();
                  return Commands.run(
                          () ->
                              Logger.recordOutput(
                                  "NoteVisualizer/ShotNotes",
                                  new Pose3d[] {
                                    startPose.interpolate(endPose, timer.get() / duration)
                                  }))
                      .until(() -> timer.hasElapsed(duration))
                      .finallyDo(
                          () -> Logger.recordOutput("NoteVisualizer/ShotNotes", new Pose3d[] {}));
                },
                Set.of())
            .ignoringDisable(true));
  }

  /** Ejects note out the back of the indexer */
  public static Command eject() {
    return new ScheduleCommand( // Branch off and exit immediately
        Commands.defer(
                () -> {
                  hasNote = false;
                  final Pose3d startPose = getIndexerPose3d();
                  final Pose3d endPose =
                      startPose.transformBy(new Transform3d(-0.5, 0, 0, new Rotation3d()));

                  final double duration =
                      startPose.getTranslation().getDistance(endPose.getTranslation()) / ejectSpeed;
                  final Timer timer = new Timer();
                  timer.start();
                  return Commands.run(
                          () ->
                              Logger.recordOutput(
                                  "NoteVisualizer/ShotNotes",
                                  new Pose3d[] {
                                    startPose.interpolate(endPose, timer.get() / duration)
                                  }))
                      .until(() -> timer.hasElapsed(duration))
                      .finallyDo(
                          () -> Logger.recordOutput("NoteVisualizer/ShotNotes", new Pose3d[] {}));
                },
                Set.of())
            .ignoringDisable(true));
  }

  /*
   * Returns the 3D pose of the indexer in field space for visualization.
   */
  private static Pose3d getIndexerPose3d() {
    Transform3d indexerTransform =
        new Transform3d(
                wristPoseSupplier.get().getTranslation(), wristPoseSupplier.get().getRotation())
            .plus(new Transform3d(-0.05, 0.0, 0.08, new Rotation3d()));
    return new Pose3d(robotPoseSupplier.get()).transformBy(indexerTransform);
  }
}
