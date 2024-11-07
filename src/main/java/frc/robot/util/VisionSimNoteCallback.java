package frc.robot.util;

/*
 * This is an interface to pass in a callback function to remove a note from a photon sim world
 */
@FunctionalInterface
public interface VisionSimNoteCallback {
  /*
   * Execute the callback function
   * @param param - Integer index to remove a note, or Pose3d to add a note
   */
  void execute(Object param);
}
