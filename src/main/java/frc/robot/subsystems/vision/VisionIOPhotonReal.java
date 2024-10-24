// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import frc.robot.Constants.VisionConstants;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIOPhotonReal implements VisionIO {
  private final PhotonCamera m_camera;

  public VisionIOPhotonReal() {
    m_camera = new PhotonCamera(VisionConstants.kApriltagCameraName);
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
}
