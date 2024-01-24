// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.net.Proxy.Type;
import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

  private static Vision _instance; 

  private PhotonCamera camera;
  private PhotonPipelineResult result;

  private boolean hasTarget = false;

  private double tx=0;
  private double ty=0;
  private double ta=0;
  
  private double x=0;
  private double y=0;
  private double z=0;

  private double[] pose;
  Transform3d bestCameraToTarget;
  private int targetID;

  private final double cameraHeight = 0.17;
  private final double cameraAngle = Units.degreesToRadians(20);
  private final double targetLowerHeight = 0.59;
  private double distance;
  static AprilTagFieldLayout aprilTagFieldLayout;

  /** Creates a new Vision. */
  public Vision() {
    camera = new PhotonCamera("photonvision");
    try{
      AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    }
    catch(Exception e) {
      AprilTagFieldLayout aprilTagFieldLayout = null;
    }
   Transform3d robotToCam = new Transform3d(new Translation3d(.5,0,.5),new Rotation3d(0,0,0));
    

   PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, robotToCam);
  }

  public static Vision getInstance(){
    if(_instance == null){
      _instance = new Vision();
    }

    return _instance;

  }

  public boolean getHasTarget(){
    return hasTarget;
  }
  public double getTx(){
    return tx;
  }

  public double getTy(){
    return ty;
  }

  // public double getTa(){
  //   return ta;
  // }

  public void getDistance() {

    //distance = Math.pow(3.3, -(ty/16.5)+0.27);
    
    distance = PhotonUtils.calculateDistanceToTargetMeters(cameraHeight, targetLowerHeight, cameraAngle, Units.degreesToRadians(ty));
    SmartDashboard.putNumber("Distance", distance);
  }

  List<Integer> tags = new ArrayList<>();
  @Override
  public void periodic() {
    
    result = camera.getLatestResult();
    hasTarget = result.hasTargets();
    List<PhotonTrackedTarget> targets = result.getTargets();
    tags.clear();
    if(hasTarget){
      PhotonTrackedTarget bestTarget = result.getBestTarget();
      tx = bestTarget.getYaw();
      ty = bestTarget.getPitch();
      ta = bestTarget.getArea();
      bestCameraToTarget = bestTarget.getBestCameraToTarget();

      x = bestCameraToTarget.getX();
      y = bestCameraToTarget.getY();
      z = bestCameraToTarget.getZ();

      targetID = bestTarget.getFiducialId();
      
    }



    getDistance();   
    SmartDashboard.putBoolean("hastarget", hasTarget);
    SmartDashboard.putNumber("tx", tx);
    SmartDashboard.putNumber("ty", ty);
    SmartDashboard.putNumber("ta", ta);

    SmartDashboard.putNumber("targetID",targetID);
      
    SmartDashboard.putNumber("x", x);
    SmartDashboard.putNumber("y", y);
    SmartDashboard.putNumber("z", z);

  }
}
