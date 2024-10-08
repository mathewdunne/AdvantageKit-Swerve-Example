// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.Optional;
import java.util.function.Supplier;

public class Vision extends SubsystemBase {
  private final VisionIO m_io;
  private final VisionIOInputs m_inputs = new VisionIOInputs();

  private final PhotonPoseEstimator m_photonEstimator;
  private double m_lastEstTimestamp = 0;

  private final AddVisionMeasurement m_addVisionMeasurementFunc;
  private final Supplier<Pose2d> m_getTrueSimPose;

  public Vision(
      VisionIO io, AddVisionMeasurement addVisionMeasurementFunc, Supplier<Pose2d> getTrueSimPose) {
    m_io = io;
    m_addVisionMeasurementFunc = addVisionMeasurementFunc;
    m_getTrueSimPose = getTrueSimPose;

    m_photonEstimator =
        new PhotonPoseEstimator(
            VisionConstants.kTagLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_RIO,
            m_io.getCamera(),
            VisionConstants.kRobotToCam);
    m_photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("Vision", m_inputs);

    PhotonPipelineResult result = VisionIOInputs.deserializePipelineResult(m_inputs.pipelineResult);

    // Check for new vision data
    double latestTimestamp = m_inputs.timestamp;
    boolean newResult = Math.abs(latestTimestamp - m_lastEstTimestamp) > 1e-5;
    if (newResult) {
      m_lastEstTimestamp = latestTimestamp;

      // Update the photonEstimator with the latest result
      Optional<EstimatedRobotPose> estimatedPose = m_photonEstimator.update(result);

      // Handle the estimated pose (e.g., feed it into a pose estimator or odometry)
      if (estimatedPose.isPresent()) {
        EstimatedRobotPose estPose = estimatedPose.get();
        Pose2d robotPose = estPose.estimatedPose.toPose2d();
        // Use estPose.estimatedPose as needed
        // For example, update your robot's pose estimator
        Matrix<N3, N1> estStdDevs = getEstimationStdDevs(robotPose, result);
        m_addVisionMeasurementFunc.addVisionMeasurement(robotPose, latestTimestamp, estStdDevs);

        Logger.recordOutput("Vision/EstimatedPose", estPose.estimatedPose.toPose2d());
      }

      // If in simulation, update the simulation field visualization
      if (Robot.isSimulation()) {
        estimatedPose.ifPresentOrElse(
            est -> {
              // Assume you have access to the Field2d object from your IO layer
              m_io.getSimDebugField()
                  .getObject("VisionEstimation")
                  .setPose(est.estimatedPose.toPose2d());
            },
            () -> {
              if (newResult) {
                m_io.getSimDebugField().getObject("VisionEstimation").setPoses();
              }
            });
      }
    }
  }

  @Override
  public void simulationPeriodic() {
    m_io.simulationPeriodic(m_getTrueSimPose.get());
  }

  public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose, PhotonPipelineResult result) {
    var estStdDevs = VisionConstants.kSingleTagStdDevs;
    var targets = result.getTargets();
    int numTags = 0;
    double avgDist = 0;
    for (var tgt : targets) {
      var tagPose = m_photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty()) continue;
      numTags++;
      avgDist +=
          tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }
    if (numTags == 0) return estStdDevs;
    avgDist /= numTags;
    // Decrease std devs if multiple targets are visible
    if (numTags > 1) estStdDevs = VisionConstants.kMultiTagStdDevs;
    // Increase std devs based on (average) distance
    if (numTags == 1 && avgDist > 4)
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

    return estStdDevs;
  }
}
