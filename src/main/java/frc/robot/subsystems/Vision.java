// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision extends SubsystemBase {
private static AprilTagFieldLayout kFieldLayout;
private static PhotonCamera camera;
private static PhotonPoseEstimator CamEstimator;
private boolean updatePoseWithVisionReadings = true;

  /** Creates a new Vision. */
  public Vision() {
    PhotonCamera camera = new PhotonCamera("Arducam_OV2311_USB_Camera");
    PhotonPoseEstimator CamEstimator = new PhotonPoseEstimator(VisionConstants.kFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, VisionConstants.camToRobot);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Camera is connected", camera.isConnected());
    if (!updatePoseWithVisionReadings) {
      return;}
  }
   public PhotonPipelineResult getCameraResult() {
    return camera.getLatestResult();
  }
    public static Optional<EstimatedRobotPose> getCameraEst() {
    var visionest = CamEstimator.update();
    return visionest;
  }
  }

