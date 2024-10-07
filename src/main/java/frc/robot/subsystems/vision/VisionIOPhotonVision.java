// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import frc.robot.Constants.VisionConstants;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.Optional;

public class VisionIOPhotonVision implements VisionIO {
  private final PhotonCamera camera;
  private final PhotonPoseEstimator photonPoseEstimator;

  public VisionIOPhotonVision() {
      camera = new PhotonCamera(VisionConstants.kCameraName);
      photonPoseEstimator = new PhotonPoseEstimator(
              VisionConstants.kTagLayout,
              PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
              camera,
              VisionConstants.kRobotToCam);
      photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
      PhotonPipelineResult result = camera.getLatestResult();

      if (result.hasTargets()) {
          Optional<EstimatedRobotPose> estimatedPose = photonPoseEstimator.update();
          if (estimatedPose.isPresent()) {
              inputs.captureTimestamp = estimatedPose.get().timestampSeconds;
              inputs.estimatedPose = estimatedPose.get().estimatedPose.toPose2d();
          }
      }
  }
}
