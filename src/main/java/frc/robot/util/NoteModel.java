// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.estimation.TargetModel;

public class NoteModel {
  public static TargetModel getNoteModel() {
    return new TargetModel(getVertices());
  }

  public static List<Translation3d> getVertices() {
    List<Translation3d> vertices = new ArrayList<>();
    vertices.add(new Translation3d(0.179000, 0.000000, 0.000000));
    vertices.add(new Translation3d(0.173151, 0.000000, 0.016070));
    vertices.add(new Translation3d(0.158341, 0.000000, 0.024620));
    vertices.add(new Translation3d(0.141500, 0.000000, 0.021651));
    vertices.add(new Translation3d(0.130508, 0.000000, 0.008551));
    vertices.add(new Translation3d(0.130508, 0.000000, -0.008551));
    vertices.add(new Translation3d(0.141500, 0.000000, -0.021651));
    vertices.add(new Translation3d(0.158341, 0.000000, -0.024620));
    vertices.add(new Translation3d(0.173151, 0.000000, -0.016070));
    vertices.add(new Translation3d(0.168205, 0.061222, 0.000000));
    vertices.add(new Translation3d(0.162709, 0.059221, 0.016070));
    vertices.add(new Translation3d(0.148792, 0.054156, 0.024620));
    vertices.add(new Translation3d(0.132967, 0.048396, 0.021651));
    vertices.add(new Translation3d(0.122637, 0.044636, 0.008551));
    vertices.add(new Translation3d(0.122637, 0.044636, -0.008551));
    vertices.add(new Translation3d(0.132967, 0.048396, -0.021651));
    vertices.add(new Translation3d(0.148792, 0.054156, -0.024620));
    vertices.add(new Translation3d(0.162709, 0.059221, -0.016070));
    vertices.add(new Translation3d(0.137122, 0.115059, 0.000000));
    vertices.add(new Translation3d(0.132641, 0.111299, 0.016070));
    vertices.add(new Translation3d(0.121296, 0.101780, 0.024620));
    vertices.add(new Translation3d(0.108395, 0.090954, 0.021651));
    vertices.add(new Translation3d(0.099975, 0.083889, 0.008551));
    vertices.add(new Translation3d(0.099975, 0.083889, -0.008551));
    vertices.add(new Translation3d(0.108395, 0.090954, -0.021651));
    vertices.add(new Translation3d(0.121296, 0.101780, -0.024620));
    vertices.add(new Translation3d(0.132641, 0.111299, -0.016070));
    vertices.add(new Translation3d(0.089500, 0.155019, 0.000000));
    vertices.add(new Translation3d(0.086576, 0.149953, 0.016070));
    vertices.add(new Translation3d(0.079171, 0.137128, 0.024620));
    vertices.add(new Translation3d(0.070750, 0.122543, 0.021651));
    vertices.add(new Translation3d(0.065254, 0.113023, 0.008551));
    vertices.add(new Translation3d(0.065254, 0.113023, -0.008551));
    vertices.add(new Translation3d(0.070750, 0.122543, -0.021651));
    vertices.add(new Translation3d(0.079171, 0.137128, -0.024620));
    vertices.add(new Translation3d(0.086576, 0.149953, -0.016070));
    vertices.add(new Translation3d(0.031083, 0.176281, 0.000000));
    vertices.add(new Translation3d(0.030067, 0.170521, 0.016070));
    vertices.add(new Translation3d(0.027496, 0.155936, 0.024620));
    vertices.add(new Translation3d(0.024571, 0.139350, 0.021651));
    vertices.add(new Translation3d(0.022662, 0.128525, 0.008551));
    vertices.add(new Translation3d(0.022662, 0.128525, -0.008551));
    vertices.add(new Translation3d(0.024571, 0.139350, -0.021651));
    vertices.add(new Translation3d(0.027496, 0.155936, -0.024620));
    vertices.add(new Translation3d(0.030067, 0.170521, -0.016070));
    vertices.add(new Translation3d(-0.031083, 0.176281, 0.000000));
    vertices.add(new Translation3d(-0.030067, 0.170521, 0.016070));
    vertices.add(new Translation3d(-0.027496, 0.155936, 0.024620));
    vertices.add(new Translation3d(-0.024571, 0.139350, 0.021651));
    vertices.add(new Translation3d(-0.022662, 0.128525, 0.008551));
    vertices.add(new Translation3d(-0.022662, 0.128525, -0.008551));
    vertices.add(new Translation3d(-0.024571, 0.139350, -0.021651));
    vertices.add(new Translation3d(-0.027496, 0.155936, -0.024620));
    vertices.add(new Translation3d(-0.030067, 0.170521, -0.016070));
    vertices.add(new Translation3d(-0.089500, 0.155019, 0.000000));
    vertices.add(new Translation3d(-0.086576, 0.149953, 0.016070));
    vertices.add(new Translation3d(-0.079171, 0.137128, 0.024620));
    vertices.add(new Translation3d(-0.070750, 0.122543, 0.021651));
    vertices.add(new Translation3d(-0.065254, 0.113023, 0.008551));
    vertices.add(new Translation3d(-0.065254, 0.113023, -0.008551));
    vertices.add(new Translation3d(-0.070750, 0.122543, -0.021651));
    vertices.add(new Translation3d(-0.079171, 0.137128, -0.024620));
    vertices.add(new Translation3d(-0.086576, 0.149953, -0.016070));
    vertices.add(new Translation3d(-0.137122, 0.115059, 0.000000));
    vertices.add(new Translation3d(-0.132641, 0.111299, 0.016070));
    vertices.add(new Translation3d(-0.121296, 0.101780, 0.024620));
    vertices.add(new Translation3d(-0.108395, 0.090954, 0.021651));
    vertices.add(new Translation3d(-0.099975, 0.083889, 0.008551));
    vertices.add(new Translation3d(-0.099975, 0.083889, -0.008551));
    vertices.add(new Translation3d(-0.108395, 0.090954, -0.021651));
    vertices.add(new Translation3d(-0.121296, 0.101780, -0.024620));
    vertices.add(new Translation3d(-0.132641, 0.111299, -0.016070));
    vertices.add(new Translation3d(-0.168205, 0.061222, 0.000000));
    vertices.add(new Translation3d(-0.162709, 0.059221, 0.016070));
    vertices.add(new Translation3d(-0.148792, 0.054156, 0.024620));
    vertices.add(new Translation3d(-0.132967, 0.048396, 0.021651));
    vertices.add(new Translation3d(-0.122637, 0.044636, 0.008551));
    vertices.add(new Translation3d(-0.122637, 0.044636, -0.008551));
    vertices.add(new Translation3d(-0.132967, 0.048396, -0.021651));
    vertices.add(new Translation3d(-0.148792, 0.054156, -0.024620));
    vertices.add(new Translation3d(-0.162709, 0.059221, -0.016070));
    vertices.add(new Translation3d(-0.179000, 0.000000, 0.000000));
    vertices.add(new Translation3d(-0.173151, 0.000000, 0.016070));
    vertices.add(new Translation3d(-0.158341, 0.000000, 0.024620));
    vertices.add(new Translation3d(-0.141500, 0.000000, 0.021651));
    vertices.add(new Translation3d(-0.130508, 0.000000, 0.008551));
    vertices.add(new Translation3d(-0.130508, 0.000000, -0.008551));
    vertices.add(new Translation3d(-0.141500, 0.000000, -0.021651));
    vertices.add(new Translation3d(-0.158341, 0.000000, -0.024620));
    vertices.add(new Translation3d(-0.173151, 0.000000, -0.016070));
    vertices.add(new Translation3d(-0.168205, -0.061222, 0.000000));
    vertices.add(new Translation3d(-0.162709, -0.059221, 0.016070));
    vertices.add(new Translation3d(-0.148792, -0.054156, 0.024620));
    vertices.add(new Translation3d(-0.132967, -0.048396, 0.021651));
    vertices.add(new Translation3d(-0.122637, -0.044636, 0.008551));
    vertices.add(new Translation3d(-0.122637, -0.044636, -0.008551));
    vertices.add(new Translation3d(-0.132967, -0.048396, -0.021651));
    vertices.add(new Translation3d(-0.148792, -0.054156, -0.024620));
    vertices.add(new Translation3d(-0.162709, -0.059221, -0.016070));
    vertices.add(new Translation3d(-0.137122, -0.115059, 0.000000));
    vertices.add(new Translation3d(-0.132641, -0.111299, 0.016070));
    vertices.add(new Translation3d(-0.121296, -0.101780, 0.024620));
    vertices.add(new Translation3d(-0.108395, -0.090954, 0.021651));
    vertices.add(new Translation3d(-0.099975, -0.083889, 0.008551));
    vertices.add(new Translation3d(-0.099975, -0.083889, -0.008551));
    vertices.add(new Translation3d(-0.108395, -0.090954, -0.021651));
    vertices.add(new Translation3d(-0.121296, -0.101780, -0.024620));
    vertices.add(new Translation3d(-0.132641, -0.111299, -0.016070));
    vertices.add(new Translation3d(-0.089500, -0.155019, 0.000000));
    vertices.add(new Translation3d(-0.086576, -0.149953, 0.016070));
    vertices.add(new Translation3d(-0.079171, -0.137128, 0.024620));
    vertices.add(new Translation3d(-0.070750, -0.122543, 0.021651));
    vertices.add(new Translation3d(-0.065254, -0.113023, 0.008551));
    vertices.add(new Translation3d(-0.065254, -0.113023, -0.008551));
    vertices.add(new Translation3d(-0.070750, -0.122543, -0.021651));
    vertices.add(new Translation3d(-0.079171, -0.137128, -0.024620));
    vertices.add(new Translation3d(-0.086576, -0.149953, -0.016070));
    vertices.add(new Translation3d(-0.031083, -0.176281, 0.000000));
    vertices.add(new Translation3d(-0.030067, -0.170521, 0.016070));
    vertices.add(new Translation3d(-0.027496, -0.155936, 0.024620));
    vertices.add(new Translation3d(-0.024571, -0.139350, 0.021651));
    vertices.add(new Translation3d(-0.022662, -0.128525, 0.008551));
    vertices.add(new Translation3d(-0.022662, -0.128525, -0.008551));
    vertices.add(new Translation3d(-0.024571, -0.139350, -0.021651));
    vertices.add(new Translation3d(-0.027496, -0.155936, -0.024620));
    vertices.add(new Translation3d(-0.030067, -0.170521, -0.016070));
    vertices.add(new Translation3d(0.031083, -0.176281, 0.000000));
    vertices.add(new Translation3d(0.030067, -0.170521, 0.016070));
    vertices.add(new Translation3d(0.027496, -0.155936, 0.024620));
    vertices.add(new Translation3d(0.024571, -0.139350, 0.021651));
    vertices.add(new Translation3d(0.022662, -0.128525, 0.008551));
    vertices.add(new Translation3d(0.022662, -0.128525, -0.008551));
    vertices.add(new Translation3d(0.024571, -0.139350, -0.021651));
    vertices.add(new Translation3d(0.027496, -0.155936, -0.024620));
    vertices.add(new Translation3d(0.030067, -0.170521, -0.016070));
    vertices.add(new Translation3d(0.089500, -0.155019, 0.000000));
    vertices.add(new Translation3d(0.086576, -0.149953, 0.016070));
    vertices.add(new Translation3d(0.079171, -0.137128, 0.024620));
    vertices.add(new Translation3d(0.070750, -0.122543, 0.021651));
    vertices.add(new Translation3d(0.065254, -0.113023, 0.008551));
    vertices.add(new Translation3d(0.065254, -0.113023, -0.008551));
    vertices.add(new Translation3d(0.070750, -0.122543, -0.021651));
    vertices.add(new Translation3d(0.079171, -0.137128, -0.024620));
    vertices.add(new Translation3d(0.086576, -0.149953, -0.016070));
    vertices.add(new Translation3d(0.137122, -0.115059, 0.000000));
    vertices.add(new Translation3d(0.132641, -0.111299, 0.016070));
    vertices.add(new Translation3d(0.121296, -0.101780, 0.024620));
    vertices.add(new Translation3d(0.108395, -0.090954, 0.021651));
    vertices.add(new Translation3d(0.099975, -0.083889, 0.008551));
    vertices.add(new Translation3d(0.099975, -0.083889, -0.008551));
    vertices.add(new Translation3d(0.108395, -0.090954, -0.021651));
    vertices.add(new Translation3d(0.121296, -0.101780, -0.024620));
    vertices.add(new Translation3d(0.132641, -0.111299, -0.016070));
    vertices.add(new Translation3d(0.168205, -0.061222, 0.000000));
    vertices.add(new Translation3d(0.162709, -0.059221, 0.016070));
    vertices.add(new Translation3d(0.148792, -0.054156, 0.024620));
    vertices.add(new Translation3d(0.132967, -0.048396, 0.021651));
    vertices.add(new Translation3d(0.122637, -0.044636, 0.008551));
    vertices.add(new Translation3d(0.122637, -0.044636, -0.008551));
    vertices.add(new Translation3d(0.132967, -0.048396, -0.021651));
    vertices.add(new Translation3d(0.148792, -0.054156, -0.024620));
    vertices.add(new Translation3d(0.162709, -0.059221, -0.016070));
    return vertices;
  }

  public static List<Pose3d> getNotePositions() {
    List<Pose3d> notePositions = new ArrayList<>();
    // sources (index 0 and 1)
    notePositions.add(new Pose3d(0.98, 0.87, 0, new Rotation3d()));
    notePositions.add(new Pose3d(15.56, 0.87, 0, new Rotation3d()));

    // blue side
    notePositions.add(new Pose3d(2.90, 4.11, 0, new Rotation3d()));
    notePositions.add(new Pose3d(2.90, 5.56, 0, new Rotation3d()));
    notePositions.add(new Pose3d(2.90, 7.01, 0, new Rotation3d()));

    // middle
    notePositions.add(new Pose3d(8.27, 4.11, 0, new Rotation3d()));
    notePositions.add(new Pose3d(8.27, 5.79, 0, new Rotation3d()));
    notePositions.add(new Pose3d(8.27, 7.47, 0, new Rotation3d()));
    notePositions.add(new Pose3d(8.27, 2.43, 0, new Rotation3d()));
    notePositions.add(new Pose3d(8.27, 0.75, 0, new Rotation3d()));

    // red side
    notePositions.add(new Pose3d(13.64, 4.11, 0, new Rotation3d()));
    notePositions.add(new Pose3d(13.64, 5.56, 0, new Rotation3d()));
    notePositions.add(new Pose3d(13.64, 7.01, 0, new Rotation3d()));

    return notePositions;
  }
}
