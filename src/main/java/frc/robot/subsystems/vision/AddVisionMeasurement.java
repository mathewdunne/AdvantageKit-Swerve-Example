package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

// interface to pass in a function to add a vision measurement
// so that the vision subsystem can add measurements to the pose estimator in the SwerveSubsystem
@FunctionalInterface
public interface AddVisionMeasurement {
    void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> covariance);
}
