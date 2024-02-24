// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {

  private static Vision _instance;

  private PhotonCamera camera;
  private PhotonPipelineResult result;

  private boolean hasTarget = false;

  private double tx = 0;
  private double ty = 0;
  private double ta = 0;

  private double x = 0;
  private double y = 0;
  private double z = 0;

  private double xPos;
  private double yPos;
  private double zPos;

  Transform3d bestCameraToTarget;
  private int targetID;

  private final double cameraHeight = 0.165;
  private final double cameraAngle = Units.degreesToRadians(19.9);
  private final double targetLowerHeight = 1.36;
  private double distance;
  static AprilTagFieldLayout aprilTagFieldLayout;
  private Rotation2d rotation;
  private Transform3d fieldToCamera;
  private PhotonPoseEstimator photonPoseEstimator;
  private Rotation2d targetToRobotRotation2d;
  private double toTargetRotation;

  private Transform3d robotToCam;
  private Pose3d robotPose;

  /** Creates a new Vision. */
  public Vision() {
    camera = new PhotonCamera("photonvision");

    try {
      AprilTagFieldLayout aprilTagFieldLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch (Exception e) {
      AprilTagFieldLayout aprilTagFieldLayout = null;
    }
    robotToCam = new Transform3d(new Translation3d(.5, 0, .5), new Rotation3d(0, 0, 0));

    photonPoseEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, robotToCam);
  }

  public static Vision getInstance() {
    if (_instance == null) {
      _instance = new Vision();
    }

    return _instance;
  }

  public boolean getHasTarget() {
    return hasTarget;
  }

  public double getTx() {
    return tx;
  }

  public double getTy() {
    return ty;
  }

  // public double targetToRobotRotation(CommandSwerveDrivetrain m_Swerve) {
  //   toTargetRotation = m_Swerve.getState().getRotation().getRadians() + tx;
  //   // Rotation2d targetYaw =
  //   // PhotonUtils.getYawToPose(m_Swerve.getPose(),aprilTagFieldLayout.getTagPose(5));
  //   return toTargetRotation;
  // }

  public Double getDistance() {
    distance =
        PhotonUtils.calculateDistanceToTargetMeters(
            cameraHeight, targetLowerHeight, cameraAngle, Units.degreesToRadians(ty));
    SmartDashboard.putNumber("Distance", distance);
    return distance;
  }

  // NOT FINISHED + need to get the right formulas
  public double getShooterAngle() {
    return getDistance() * 123;
  }

  public double getShooterSpeed() {
    return getDistance() * 123;
  }

  List<Integer> tags = new ArrayList<>();

  @Override
  public void periodic() {

    result = camera.getLatestResult();
    hasTarget = result.hasTargets();
    // if (result.getMultiTagResult().estimatedPose.isPresent){
    //   fieldToCamera = result.getMultiTagResult().estimatedPose.best;

    // }
    List<PhotonTrackedTarget> targets = result.getTargets();
    tags.clear();

    if (hasTarget) {
      // photonPoseEstimator.update();
      PhotonTrackedTarget bestTarget = result.getBestTarget();
      tx = bestTarget.getYaw();
      ty = bestTarget.getPitch();
      ta = bestTarget.getArea();
      bestCameraToTarget = bestTarget.getBestCameraToTarget();
      Pose3d bestTagPose = aprilTagFieldLayout.getTagPose(bestTarget.getFiducialId()).orElse(null);
      robotPose =
          PhotonUtils.estimateFieldToRobotAprilTag(bestCameraToTarget, bestTagPose, robotToCam);

      xPos = robotPose.getTranslation().getX();
      yPos = robotPose.getTranslation().getY();

      x = bestCameraToTarget.getX();
      y = bestCameraToTarget.getY();
      z = bestCameraToTarget.getZ();

      targetID = bestTarget.getFiducialId();

      try {
        var targetpose = aprilTagFieldLayout.getTagPose(5);
        SmartDashboard.putNumber("apriltagheight", targetpose.get().getZ());
      } catch (Exception e) {
      }
    }

    // targetToRobotRotation = targetToRobotRotation();
    getDistance();
    // SmartDashboard.putNumber("rotation pos of target", targetToRobotRotation.getRadians());
    // SmartDashboard.putBoolean("hastarget", hasTarget);
    // SmartDashboard.putNumber("tx", tx);
    // SmartDashboard.putNumber("ty", ty);
    // SmartDashboard.putNumber("ta", ta);
    // SmartDashboard.putNumber("robotPose X: ", xPos);
    // SmartDashboard.putNumber("robotPose y: ", yPos);

    // SmartDashboard.putNumber("targetID", targetID);

    // SmartDashboard.putNumber("x", x);
    // SmartDashboard.putNumber("y", y);
    // SmartDashboard.putNumber("z", z);
  }
}
