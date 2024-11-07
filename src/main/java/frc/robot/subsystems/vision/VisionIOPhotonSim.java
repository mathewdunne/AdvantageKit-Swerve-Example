// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;
import frc.robot.util.NoteModel;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
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

  private final List<VisionTargetSim> m_noteTargets;

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
    m_noteTargets = new ArrayList<>();
    for (int i = 0; i < NoteModel.getNotePositions().size(); i++) {
      m_noteTargets.add(new VisionTargetSim(NoteModel.getNotePositions().get(i), noteModel));
    }
    m_visionSimWorldGamePieces.addVisionTargets(
        "note", m_noteTargets.toArray(new VisionTargetSim[0]));

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
    PhotonPipelineResult resultShooter = m_shooterCamera.getLatestResult();
    inputs.apriltagCamTimestamp = resultShooter.getTimestampSeconds();
    inputs.apriltagCamPipelineResult = VisionIOInputs.serializePipelineResult(resultShooter);

    PhotonPipelineResult resultIntake = m_intakeCamera.getLatestResult();
    inputs.intakeCamTimestamp = resultIntake.getTimestampSeconds();
    inputs.intakeCamPipelineResult = VisionIOInputs.serializePipelineResult(resultIntake);
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

  /*
   * Adds or removes a note from the simulation world
   * @param param - Integer index to remove a note, or Pose3d to add a note
   */
  public void manageNotesInSimulation(Object param) {
    // remove a note
    if (param instanceof Integer) {
      int index = (Integer) param;
      if (index >= 0 && index < m_noteTargets.size()) {
        VisionTargetSim targetToRemove = m_noteTargets.get(index);
        m_visionSimWorldGamePieces.removeVisionTargets(targetToRemove);
      } else {
        System.out.println("[VisionSim] Invalid index for removal.");
      }

      // add a note
    } else if (param instanceof Pose3d) {
      Pose3d notePose = (Pose3d) param;
      VisionTargetSim noteModel =
          new VisionTargetSim(
              new Pose3d(
                  notePose.getX(), notePose.getY(), Units.inchesToMeters(1.0), new Rotation3d()),
              NoteModel.getNoteModel());
      m_noteTargets.add(noteModel);
      m_visionSimWorldGamePieces.addVisionTargets("note", noteModel);

      // error
    } else {
      System.out.println("[VisionSim] Invalid parameter type for manageNotesInSimulation");
    }
  }
}
