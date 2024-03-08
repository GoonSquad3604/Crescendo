// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagPoseEstimate;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.apriltag.jni.AprilTagJNI;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
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
  private double txSpeaker4 = 0;
  private double txSpeaker14 =0;
  

  private double ty = 0;
  private double ta = 0;

  private double x = 0;
  private double y = 0;
  private double z = 0;

  private double xPos;
  private double yPos;
  private double zPos;
  private double angle;
  Transform3d bestCameraToTarget;
  private int targetID;

  private final double cameraHeight = 0.3048;
  private final double cameraAngle = Units.degreesToRadians(18.8);
  private final double targetLowerHeight = 1.3208;
  private double distance;
  static AprilTagFieldLayout aprilTagFieldLayout;
  private Rotation2d rotation;
  private Transform3d fieldToCamera;
  private PhotonPoseEstimator photonPoseEstimator;
  private Rotation2d targetToRobotRotation2d;
  private double toTargetRotation;
  Pose3d bestTagPose;
  private Transform3d robotToCam;
  private Pose3d robotPose;
private boolean has4;
private boolean has14;

public static final Translation2d SPEAKER_RED = new Translation2d(16.579 - .1016, 5.5478);
  public static final Translation2d SPEAKER_BLUE = new Translation2d(.1016, 5.5478);
  private Translation2d speakerTranslation;
  private double speakerDist;
  public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4,4,8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(.5,.5,1);

  /** Creates a new Vision. */
  public Vision() {
    
    if(setSpeakerTranslation()) speakerTranslation = SPEAKER_RED;
    else speakerTranslation = SPEAKER_BLUE;
    // speakerTranslation = (alliance.isEmpty() || alliance.get() == Alliance.Blue) ? SPEAKER_BLUE : SPEAKER_RED;
    // speakerTranslation = SPEAKER_RED;
    camera = new PhotonCamera("photonvision1");


    try {
      aprilTagFieldLayout =
      AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);

    } catch (Exception e) {
      aprilTagFieldLayout = null;
    }
    robotToCam = new Transform3d(new Translation3d(.5, 0, .5), new Rotation3d(0, Math.toRadians(15), 0));

    photonPoseEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCam);

    photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  public static Vision getInstance() {
    if (_instance == null) {
      _instance = new Vision();
    }

    return _instance;
  }
  public boolean setSpeakerTranslation() {
        var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }  
        return false;
        }

  public Pose2d robotPose() {
    if(!getHasTarget()) return null;
    return robotPose.toPose2d();
  }
  
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    photonPoseEstimator.setReferencePose(robotPose);
    return photonPoseEstimator.update();

  }
  public Matrix<N3, N1> getEstimationStdDevs (Pose2d estimatedPose) {
    var estStdDevs = kSingleTagStdDevs;
    var targets = camera.getLatestResult().getTargets();
    int numTags = 0;
    double avgDist = 0;
    for(var tar : targets) {
        var tagPose = photonPoseEstimator.getFieldTags().getTagPose(tar.getFiducialId());
        if(tagPose.isEmpty()) continue; 
          numTags++;
          avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());

    }
    if(numTags == 0) return estStdDevs;
    avgDist /= numTags;
    if(numTags >1 ) estStdDevs = kMultiTagStdDevs;
    if (numTags ==1 && avgDist >4) 
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    else estStdDevs = estStdDevs.times(1+ (avgDist * avgDist/30));

    return estStdDevs;

  }


  public boolean getHasTarget() {
    return hasTarget;
  }

  public double getTx() {
    return tx;
  }
  public double getTxSpeaker() {
    if(setSpeakerTranslation())    return txSpeaker4;
    return txSpeaker14;
  }

  public double getTy() {
    return ty;
  }
  public boolean has4() {
    return has4;
  }
  public boolean has14() {
    return has14;
  }
  public double getX() {
    return x;
  }
  public Translation2d getSpeakerTranslation() {
    return speakerTranslation;
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
    //if(Math.atan(60/(getDistance()-.2413))<=56&&Math.atan(60/getDistance()-.2413)>=10)
    // return Math.toDegrees(Math.atan(1.524/(getX()-.27-.2159)));
    return Math.toDegrees(Math.atan(1.524/(speakerDist-.2413))) -4;

    //return 56;
  }

  
  
  

  public double getShooterSpeed() {
    return getDistance() * 123;
  }

  List<Integer> tags = new ArrayList<>();

  @Override
  public void periodic() {

    result = camera.getLatestResult();
    hasTarget = result.hasTargets();
    if (result.getMultiTagResult().estimatedPose.isPresent){
      fieldToCamera = result.getMultiTagResult().estimatedPose.best;

    }
    List<PhotonTrackedTarget> targets = result.getTargets();
    tags.clear();
    if (hasTarget) {
      for(PhotonTrackedTarget x: targets){
        if(x.getFiducialId()==4){
          txSpeaker4 = x.getYaw();
        }
      }
      for(PhotonTrackedTarget x: targets){
        if(x.getFiducialId()==14){
          txSpeaker14 = x.getYaw();
        }
      }
      // photonPoseEstimator.update();
      PhotonTrackedTarget bestTarget = result.getBestTarget();
      
      if(bestTarget.getFiducialId() ==4) 
      {txSpeaker4 = bestTarget.getYaw(); 
        has4 = true;
      }
      else has4 = false;
      tx = bestTarget.getYaw();
      ty = bestTarget.getPitch();
      ta = bestTarget.getArea();
      bestCameraToTarget = bestTarget.getBestCameraToTarget();
       bestTagPose = aprilTagFieldLayout.getTagPose(bestTarget.getFiducialId()).orElse(null);
      // if(getHasTarget()){
      //   photonPoseEstimator.setReferencePose(bestTagPose);
      //   robotPose = 
      //  //photonPoseEstimator.update().get().estimatedPose;
      //    PhotonUtils.estimateFieldToRobotAprilTag(bestCameraToTarget, bestTagPose, robotToCam);
      // }
      
      // xPos = robotPose.getTranslation().getX();
      // yPos = robotPose.getTranslation().getY();    

      x = bestCameraToTarget.getX();
      y = bestCameraToTarget.getY();
      z = bestCameraToTarget.getZ();

      // targetID = bestTarget.getFiducialId();
      // var robotTranslation = new Translation2d(robotPose.getX(), robotPose.getY());
      // speakerDist = robotTranslation.getDistance(speakerTranslation);
      
    }
    // targetToRobotRotation = targetToRobotRotation();
    getDistance();
        SmartDashboard.putNumber("angleto", getShooterAngle());

    // SmartDashboard.putNumber("rotation pos of target", targetToRobotRotation.getRadians());
    SmartDashboard.putBoolean("hastarget", hasTarget);
    SmartDashboard.putNumber("tx", tx);
    // SmartDashboard.putNumber("ty", ty);
    // SmartDashboard.putNumber("ta", ta);
    // SmartDashboard.putNumber("robotPose X: ", xPos);
    // SmartDashboard.putNumber("robotPose y: ", yPos);

    SmartDashboard.putNumber("targetID", targetID);
    // SmartDashboard.putNumber("txSpeaker", txSpeaker);
    SmartDashboard.putNumber("x", x);
    SmartDashboard.putNumber("y", y);
    SmartDashboard.putNumber("z", z);
  }
}
