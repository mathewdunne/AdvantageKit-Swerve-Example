// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;
import frc.robot.util.NoteModel;
import java.io.IOException;
import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIOPhotonSim implements VisionIO {
  private final PhotonCamera m_shooterCamera;
  private final PhotonCamera m_intakeCamera;

  private final PhotonCameraSim m_shooterCameraSim;
  private final PhotonCameraSim m_intakeCameraSim;

  private final VisionSystemSim m_visionSimWorldApriltags;
  private final VisionSystemSim m_visionSimWorldGamePieces;

  // Simulated camera streams are CPU intensive and can be disabled when not needed
  boolean renderSim = true;

  public VisionIOPhotonSim() {
    m_shooterCamera = new PhotonCamera(VisionConstants.kApriltagCameraName);
    m_intakeCamera = new PhotonCamera(VisionConstants.kIntakeCameraName);

    // Create the vision system simulation which handles cameras and targets on the field.
    m_visionSimWorldApriltags = new VisionSystemSim("aprilTags");
    // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
    m_visionSimWorldApriltags.addAprilTags(VisionConstants.kTagLayout);

    // Different simulated world with game piece targets
    m_visionSimWorldGamePieces = new VisionSystemSim("gamePieces");
    TargetModel noteModel = NoteModel.getNoteModel();
    VisionTargetSim[] noteTargets = new VisionTargetSim[NoteModel.getNotePositions().size()];
    for (int i = 0; i < noteTargets.length; i++) {
      noteTargets[i] = new VisionTargetSim(NoteModel.getNotePositions().get(i), noteModel);
    }
    m_visionSimWorldGamePieces.addVisionTargets("note", noteTargets);

    // Create simulated camera properties. These can be set to mimic your actual camera.
    SimCameraProperties shooterCameraProps;
    try {
      shooterCameraProps =
          new SimCameraProperties(
              "./photonvision/exports/cameras/copperShooterCamera.json", 1280, 720);
      System.out.println("[VisionSim] Loaded shooter camera calibration file");
    } catch (IOException e) {
      System.out.println(
          "[VisionSim] Failed to load shooter camera calibration file, using default values");
      shooterCameraProps = new SimCameraProperties();
    }
    shooterCameraProps.setFPS(15);

    SimCameraProperties intakeCameraProps = new SimCameraProperties();
    intakeCameraProps.setCalibration(960, 720, new Rotation2d(Units.degreesToRadians(90)));
    intakeCameraProps.setFPS(15);

    // Create a PhotonCameraSim which will update the PhotonCamera's values with visible targets.
    m_shooterCameraSim = new PhotonCameraSim(m_shooterCamera, shooterCameraProps);
    m_intakeCameraSim = new PhotonCameraSim(m_intakeCamera, intakeCameraProps);
    // Add the simulated camera to view the targets on this simulated field.
    m_visionSimWorldApriltags.addCamera(m_shooterCameraSim, VisionConstants.kRobotToApriltagCam);
    m_visionSimWorldGamePieces.addCamera(m_intakeCameraSim, VisionConstants.kRobotToIntakeCam);

    m_shooterCameraSim.enableRawStream(renderSim);
    m_shooterCameraSim.enableProcessedStream(renderSim);
    m_shooterCameraSim.enableDrawWireframe(renderSim);
    m_intakeCameraSim.enableRawStream(renderSim);
    m_intakeCameraSim.enableProcessedStream(renderSim);
    m_intakeCameraSim.enableDrawWireframe(renderSim);
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

  /** Updates the simulation with the true robot pose. Must be called from a subsystem */
  @Override
  public void simulationPeriodic(Pose2d simTruePose) {
    if (Robot.isSimulation()) {
      m_visionSimWorldApriltags.update(simTruePose);
      m_visionSimWorldGamePieces.update(simTruePose);
    }
  }
}
