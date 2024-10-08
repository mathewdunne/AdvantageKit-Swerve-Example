// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonPipelineResult.APacketSerde;

public interface VisionIO {

  public static class VisionIOInputs implements LoggableInputs {

    public byte[] pipelineResult = new byte[] {};

    // Serializer/deserializer for PhotonPipelineResult
    private static final APacketSerde serde = new APacketSerde();

    public void toLog(LogTable table) {
      table.put("PipelineResult", pipelineResult);
    }

    public void fromLog(LogTable table) {
      pipelineResult = table.get("PipelineResult", pipelineResult);
    }

    /**  Method to pack a PhotonPipelineResult into a byte array */
    public static byte[] serializePipelineResult(PhotonPipelineResult result) {
      Packet packet = new Packet(result.getPacketSize());
      serde.pack(packet, result);
      return packet.getData();
    }

    /** Method to unpack a byte array into a PhotonPipelineResult */
    public static PhotonPipelineResult deserializePipelineResult(byte[] data) {
      Packet packet = new Packet(data);
      return serde.unpack(packet);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(VisionIOInputs inputs) {}

}

