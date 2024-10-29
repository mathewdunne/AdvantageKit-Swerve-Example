package frc.robot.util;

import frc.robot.Constants;

/*
 * This is an interface to pass in a callback function to set the aimbot mode
 * so that commands can set the aimbot mode controlled by the master drive command
 */
@FunctionalInterface
public interface SetAimDriveModeCallback {
  /** Execute the callback function */
  void execute(Constants.AimDriveMode mode);
}
