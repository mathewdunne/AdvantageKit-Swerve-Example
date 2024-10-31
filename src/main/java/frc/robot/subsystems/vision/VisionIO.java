// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants.VisionConstants;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.PhotonCamera;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonPipelineResult.APacketSerde;

public interface VisionIO {

  public static class VisionIOInputs implements LoggableInputs {

    public double apriltagCamTimestamp = 0.0;
    public byte[] apriltagCamPipelineResult = new byte[] {};

    public double intakeCamTimestamp = 0.0;
    public byte[] intakeCamPipelineResult = new byte[] {};

    // Serializer/deserializer for PhotonPipelineResult
    private static final APacketSerde serde = new APacketSerde();

    public void toLog(LogTable table) {
      table.put("AprilTagCamTimestamp", apriltagCamTimestamp);
      table.put("AprilTagCamPipelineResult", apriltagCamPipelineResult);
      table.put("IntakeCamTimestamp", intakeCamTimestamp);
      table.put("IntakeCamPipelineResult", intakeCamPipelineResult);
    }

    public void fromLog(LogTable table) {
      apriltagCamTimestamp = table.get("AprilTagCamTimestamp", apriltagCamTimestamp);
      apriltagCamPipelineResult = table.get("AprilTagCamPipelineResult", apriltagCamPipelineResult);
      intakeCamTimestamp = table.get("IntakeCamTimestamp", intakeCamTimestamp);
      intakeCamPipelineResult = table.get("IntakeCamPipelineResult", intakeCamPipelineResult);
    }

    /** Method to pack a PhotonPipelineResult into a byte array */
    public static byte[] serializePipelineResult(PhotonPipelineResult result) {
      Packet packet = new Packet(result.getPacketSize());
      serde.pack(packet, result);
      return packet.getData();
    }

    /** Method to unpack a byte array into a PhotonPipelineResult */
    public static PhotonPipelineResult deserializePipelineResult(byte[] data, double timestamp) {
      Packet packet = new Packet(data);
      PhotonPipelineResult result = serde.unpack(packet);
      result.setTimestampSeconds(timestamp);
      return result;
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(VisionIOInputs inputs) {}

  /** Gets the PhotonCamera object. Must be implemented in both simulation and real robot classes */
  public default PhotonCamera getApriltagCamera() {
    return new PhotonCamera(VisionConstants.kApriltagCameraName);
  }

  // ONLY NEEDED FOR SIMULATION
  /** Updates the simulation with the true robot pose. Must be called from a subsystem */
  public default void simulationPeriodic(Pose2d simTruePose) {
    throw new UnsupportedOperationException(
        "simulationPeriodic is not supported outside of simulation");
  }

  /** A Field2d for visualizing our robot and objects on the field. */
  public default Field2d getSimDebugField() {
    throw new UnsupportedOperationException(
        "getSimDebugField is not supported outside of simulation");
  }
}
