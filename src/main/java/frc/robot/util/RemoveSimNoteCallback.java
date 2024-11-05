package frc.robot.util;

/*
 * This is an interface to pass in a callback function to remove a note from a photon sim world
 */
@FunctionalInterface
public interface RemoveSimNoteCallback {
  /** Execute the callback function */
  void execute(int noteIndex);
}
