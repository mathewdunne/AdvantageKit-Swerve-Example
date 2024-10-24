// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;
import java.io.IOException;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIOPhotonSim implements VisionIO {
  private final PhotonCamera m_camera;
  private final PhotonCameraSim m_cameraSim;
  private final VisionSystemSim m_visionSim;

  // Simulated camera streams are CPU intensive and can be disabled when not needed
  boolean renderSim = true;

  public VisionIOPhotonSim() {
    m_camera = new PhotonCamera(VisionConstants.kApriltagCameraName);
    // Create the vision system simulation which handles cameras and targets on the field.
    m_visionSim = new VisionSystemSim("main");
    // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
    m_visionSim.addAprilTags(VisionConstants.kTagLayout);
    // Create simulated camera properties. These can be set to mimic your actual camera.
    SimCameraProperties cameraProp;
    try {
      cameraProp =
          new SimCameraProperties("./photonvision/exports/copperShooterCamera.json", 1280, 720);
      System.out.println("[VisionSim] Loaded camera calibration file");
    } catch (IOException e) {
      System.out.println(
          "[VisionSim] Failed to load camera calibration file, using default values");
      cameraProp = new SimCameraProperties();
    }
    cameraProp.setFPS(15);
    // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
    // targets.
    m_cameraSim = new PhotonCameraSim(m_camera, cameraProp);
    // Add the simulated camera to view the targets on this simulated field.
    m_visionSim.addCamera(m_cameraSim, VisionConstants.kRobotToApriltagCam);

    m_cameraSim.enableRawStream(renderSim);
    m_cameraSim.enableProcessedStream(renderSim);
    m_cameraSim.enableDrawWireframe(renderSim);
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
