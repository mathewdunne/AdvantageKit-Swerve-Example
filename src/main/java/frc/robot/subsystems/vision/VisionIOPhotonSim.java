// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Robot;
import frc.robot.Constants.VisionConstants;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIOPhotonSim implements VisionIO {
  private final PhotonCamera m_camera;
  private final PhotonCameraSim m_cameraSim;
  private final VisionSystemSim m_visionSim;

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

  @Override
  public PhotonCamera getCamera() {
    return m_camera;
  }

  /** Updates the simulation with the true robot pose. Must be called from a subsystem */
  @Override
  public void simulationPeriodic(Pose2d simTruePose) {
    if (Robot.isSimulation()) {
      m_visionSim.update(simTruePose);
    }
  }

  /** A Field2d for visualizing our robot and objects on the field. */
  public Field2d getSimDebugField() {
    return m_visionSim.getDebugField();
  }
}
