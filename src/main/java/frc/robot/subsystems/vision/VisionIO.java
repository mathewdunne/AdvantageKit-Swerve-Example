// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface VisionIO {

  public static class VisionIOInputs implements LoggableInputs {
    public double captureTimestamp = 0.0;
    public Pose2d estimatedPose = new Pose2d();

    public void toLog(LogTable table) {
        table.put("CaptureTimestamp", captureTimestamp);
        table.put("EstimatedPose", estimatedPose);
    }

    public void fromLog(LogTable table) {
        captureTimestamp = table.get("CaptureTimestamp", captureTimestamp);
        estimatedPose = table.get("EstimatedPose", estimatedPose);
    }
}

  public default void updateInputs(VisionIOInputs inputs) {}
}
