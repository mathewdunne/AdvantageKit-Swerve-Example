// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import frc.robot.Constants.VisionConstants;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIOPhotonReal implements VisionIO {
  private final PhotonCamera m_shooterCamera;
  private final PhotonCamera m_intakeCamera;

  public VisionIOPhotonReal() {
    m_shooterCamera = new PhotonCamera(VisionConstants.kApriltagCameraName);
    m_intakeCamera = new PhotonCamera(VisionConstants.kIntakeCameraName);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    PhotonPipelineResult result = m_shooterCamera.getLatestResult();
    inputs.apriltagCamTimestamp = result.getTimestampSeconds();
    inputs.apriltagCamPipelineResult = VisionIOInputs.serializePipelineResult(result);

    result = m_intakeCamera.getLatestResult();
    inputs.intakeCamTimestamp = result.getTimestampSeconds();
    inputs.intakeCamPipelineResult = VisionIOInputs.serializePipelineResult(result);
  }

  @Override
  public PhotonCamera getApriltagCamera() {
    return m_shooterCamera;
  }
}
