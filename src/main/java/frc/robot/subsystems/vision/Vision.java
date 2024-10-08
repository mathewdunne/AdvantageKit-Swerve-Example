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
  private double m_lastEstTimestamp = 0;
  private final AddVisionMeasurement m_addVisionMeasurementFunc;


  public Vision(VisionIO io, AddVisionMeasurement addVisionMeasurementFunc) {
    m_addVisionMeasurementFunc = addVisionMeasurementFunc;
    m_io = io;

    }
  }

    /** Returns the latest estimated robot pose from vision data */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
      // Method left blank
      return Optional.empty();
    }
  
    /** Returns the latest PhotonPipelineResult */
    public PhotonPipelineResult getLatestResult() {
      // Method left blank
      return new PhotonPipelineResult();
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
