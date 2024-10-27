package frc.robot.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/*
 * This is an interface to pass in a callback function to add a vision measurement
 * so that the vision subsystem can add measurements to the pose estimator in the SwerveSubsystem
 */
@FunctionalInterface
public interface AddVisionMeasurementCallback {
  /**
   * Execute the callback function
   *
   * @param pose
   * @param timestamp
   * @param covariance
   */
  void execute(Pose2d pose, double timestamp, Matrix<N3, N1> covariance);
}
