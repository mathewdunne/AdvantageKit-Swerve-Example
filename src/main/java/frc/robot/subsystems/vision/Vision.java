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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;

import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

public class Vision extends SubsystemBase {
  private final VisionIO m_io;
  private final VisionIOInputs m_inputs = new VisionIOInputs();

  private final PhotonCamera m_camera;
  private final PhotonPoseEstimator m_photonEstimator;
  private double m_lastEstTimestamp = 0;
  private final AddVisionMeasurement m_addVisionMeasurementFunc;

  // Simulation
  private PhotonCameraSim m_cameraSim;
  private VisionSystemSim m_visionSim;
  private Supplier<Pose2d> m_getRobotSimPoseFunc;

  public Vision(VisionIO io, AddVisionMeasurement addVisionMeasurementFunc) {
    m_addVisionMeasurementFunc = addVisionMeasurementFunc;
    m_io = io;

    m_camera = new PhotonCamera(VisionConstants.kCameraName);

    m_photonEstimator =
        new PhotonPoseEstimator(
            VisionConstants.kTagLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            m_camera,
            VisionConstants.kRobotToCam);
    m_photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    // ----- Simulation
    if (Constants.kCurrentMode == Constants.Mode.SIM) {
      // Create the vision system simulation which handles cameras and targets on the field.
      m_visionSim = new VisionSystemSim("main");
      // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
      m_visionSim.addAprilTags(VisionConstants.kTagLayout);
      // Create simulated camera properties. These can be set to mimic your actual camera.
      SimCameraProperties cameraProp = new SimCameraProperties();
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
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    var visionEst = m_photonEstimator.update();
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
  public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
    var estStdDevs = VisionConstants.kSingleTagStdDevs;
    var targets = getLatestResult().getTargets();
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

  // ----- Simulation

  @Override
  public void simulationPeriodic() {
    m_visionSim.update(m_getRobotSimPoseFunc.get());
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("Vision", m_inputs);
    m_addVisionMeasurementFunc(m_inputs.captureTimestamp, m_inputs.estimatedPose, // not sure what to do here
  }

  // /** Reset pose history of the robot in the vision system simulation. */
  // public void resetSimPose(Pose2d pose) {
  //   if (Robot.isSimulation()) visionSim.resetRobotPose(pose);
  // }

  /** A Field2d for visualizing our robot and objects on the field. */
  public Field2d getSimDebugField() {
    if (!Robot.isSimulation()) return null;
    return m_visionSim.getDebugField();
  }
}
