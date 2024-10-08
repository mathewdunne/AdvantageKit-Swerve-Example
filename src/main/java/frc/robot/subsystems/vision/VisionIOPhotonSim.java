// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.Optional;

public class VisionIOPhotonSim implements VisionIO {
  private final PhotonCamera m_camera;
  private final PhotonCameraSim m_cameraSim;
  private final VisionSystemSim m_visionSim;
  private double m_lastEstTimestamp = 0;

  public VisionIOPhotonSim() {
    m_camera = new PhotonCamera(VisionConstants.kCameraName);
    // Create the vision system simulation which handles cameras and targets on the field.
    m_visionSim = new VisionSystemSim("main");
    // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
    m_visionSim.addAprilTags(VisionConstants.kTagLayout);
    // Create simulated camera properties. These can be set to mimic your actual camera.
    var cameraProp = new SimCameraProperties();
    cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
    cameraProp.setCalibError(0.35, 0.10);
    cameraProp.setFPS(15);
    cameraProp.setAvgLatencyMs(50);
    cameraProp.setLatencyStdDevMs(15);
    // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
    // targets.
    m_cameraSim = new PhotonCameraSim(m_camera, cameraProp);
    // Add the simulated camera to view the targets on this simulated field.
    m_visionSim.addCamera(m_cameraSim, VisionConstants.kRobotToCam);

    m_cameraSim.enableDrawWireframe(true);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    PhotonPipelineResult result = m_camera.getLatestResult();
    inputs.timestamp = result.getTimestampSeconds();
    inputs.pipelineResult = VisionIOInputs.serializePipelineResult(result);
  }

  public PhotonPipelineResult getLatestResult() {
    return m_camera.getLatestResult();
  }

  /**
   * The latest estimated robot pose on the field from vision data. This may be empty. This should
   * only be called once per loop.
   *
   * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
   *     used for estimation.
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(PhotonPoseEstimator photonEstimator) {
    var visionEst = photonEstimator.update();
    double latestTimestamp = m_camera.getLatestResult().getTimestampSeconds();
    boolean newResult = Math.abs(latestTimestamp - m_lastEstTimestamp) > 1e-5;
    if (Robot.isSimulation()) {
      visionEst.ifPresentOrElse(
          est ->
              getSimDebugField()
                  .getObject("VisionEstimation")
                  .setPose(est.estimatedPose.toPose2d()),
          () -> {
            if (newResult) getSimDebugField().getObject("VisionEstimation").setPoses();
          });
    }
    if (newResult) m_lastEstTimestamp = latestTimestamp;
    return visionEst;
  }

  /**
   * The standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for use
   * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
   * This should only be used when there are targets visible.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   */
  public Matrix<N3, N1> getEstimationStdDevs(
      Pose2d estimatedPose, PhotonPoseEstimator photonEstimator) {
    var estStdDevs = VisionConstants.kSingleTagStdDevs;
    var targets = getLatestResult().getTargets();
    int numTags = 0;
    double avgDist = 0;
    for (var tgt : targets) {
      var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
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

  // ----- Simulation

  public void simulationPeriodic(Pose2d robotSimPose) {
    m_visionSim.update(robotSimPose);
  }

  /** Reset pose history of the robot in the vision system simulation. */
  public void resetSimPose(Pose2d pose) {
    m_visionSim.resetRobotPose(pose);
  }

  /** A Field2d for visualizing our robot and objects on the field. */
  public Field2d getSimDebugField() {
    return m_visionSim.getDebugField();
  }
}
