// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import frc.robot.util.AddVisionMeasurementCallback;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class Vision extends SubsystemBase {
  private final VisionIO m_io;
  private final VisionIOInputs m_inputs = new VisionIOInputs();

  private final PhotonPoseEstimator m_photonEstimator;
  private double m_lastEstTimestampApriltags = 0;

  private final AddVisionMeasurementCallback m_addVisionMeasurementCallback;
  private Supplier<Pose2d> m_getTrueSimPose;

  public Vision(VisionIO io, AddVisionMeasurementCallback addVisionMeasurementCallback) {
    m_io = io;
    m_addVisionMeasurementCallback = addVisionMeasurementCallback;

    m_photonEstimator =
        new PhotonPoseEstimator(
            VisionConstants.kTagLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            m_io.getApriltagCamera(),
            VisionConstants.kRobotToApriltagCam);
    m_photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("Vision", m_inputs);

    // Check for new apriltag data
    PhotonPipelineResult aprilTagResult =
        VisionIOInputs.deserializePipelineResult(
            m_inputs.apriltagCamPipelineResult, m_inputs.apriltagCamTimestamp);

    double latestTimestamp = m_inputs.apriltagCamTimestamp;
    boolean newResult = Math.abs(latestTimestamp - m_lastEstTimestampApriltags) > 1e-5;
    if (newResult) {
      m_lastEstTimestampApriltags = latestTimestamp;

      // Update the photonEstimator with the latest result
      Optional<EstimatedRobotPose> estimatedPose = m_photonEstimator.update(aprilTagResult);

      // Handle the estimated pose (e.g., feed it into a pose estimator or odometry)
      if (estimatedPose.isPresent()) {
        EstimatedRobotPose estPose = estimatedPose.get();
        Pose2d robotPose = estPose.estimatedPose.toPose2d();
        Matrix<N3, N1> estStdDevs = getEstimationStdDevs(robotPose, aprilTagResult);
        m_addVisionMeasurementCallback.execute(robotPose, latestTimestamp, estStdDevs);

        Logger.recordOutput("Vision/VisionEstimatedPose", estPose.estimatedPose.toPose2d());
      }

      // Log the detected AprilTags
      Pose3d[] allTagPoses =
          aprilTagResult.getTargets().stream()
              .filter(target -> target.getFiducialId() != -1)
              .map(target -> VisionConstants.kTagLayout.getTagPose(target.getFiducialId()))
              .filter(Optional::isPresent)
              .map(Optional::get)
              .toArray(Pose3d[]::new);

      Logger.recordOutput("Vision/AllTagPoses", allTagPoses);

      // Log pitch and yaw of all detected notes
      PhotonPipelineResult noteResult =
          VisionIOInputs.deserializePipelineResult(
              m_inputs.intakeCamPipelineResult, m_inputs.intakeCamTimestamp);
      double[] pitches = new double[noteResult.getTargets().size()];
      double[] yaws = new double[noteResult.getTargets().size()];
      for (int i = 0; i < noteResult.getTargets().size(); i++) {
        pitches[i] = Units.degreesToRadians(noteResult.getTargets().get(i).getPitch());
        yaws[i] = Units.degreesToRadians(noteResult.getTargets().get(i).getYaw());
      }
      Logger.recordOutput("Vision/IntakeCamPitches", pitches);
      Logger.recordOutput("Vision/IntakeCamYaws", yaws);
    }
  }

  public void setSimTruePoseSupplier(Supplier<Pose2d> getTrueSimPose) {
    m_getTrueSimPose = getTrueSimPose;
  }

  @Override
  public void simulationPeriodic() {
    if (m_getTrueSimPose == null) return;
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
