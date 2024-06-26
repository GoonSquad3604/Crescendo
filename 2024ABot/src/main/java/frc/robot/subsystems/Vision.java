package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.io.IOException;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
  private final PhotonCamera m_camera;
  private AprilTagFieldLayout m_layout;
  private final Transform3d m_offset;

  private PhotonPoseEstimator m_estimator;

  // "2.5": 44 in (14 in)
  // "5.5": 80 in (14 in)
  // "7.5": 104 in (14 in)
  // "10.5": 140 in (14 in diff)
  // "14.5": 188 in (14 in diff)

  /**
   * Subsystem that handles a PhotonVision attached camera.
   *
   * @param cameraName The name of the camera, in the UI.
   * @param robotToCamera The transformation from the center of the robot to the center of the
   *     camera lens.
   */
  public Vision(String cameraName, Transform3d robotToCamera) {
    m_camera = new PhotonCamera(cameraName);
    m_offset = robotToCamera;

    try {
      m_layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
      m_estimator =
          new PhotonPoseEstimator(
              m_layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_camera, robotToCamera);
      m_estimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);

    } catch (IOException e) {
      System.err.println(e.getMessage());
      System.exit(1);
    }
  }

  public double getDistOfTag(Pose2d prevPose) {
    try {
      Pose2d target;
      Pose2d pose = prevPose;
      double distance;
      if (getBestTarget() != null) {
        if (getHasTarget() && getBestTarget() != null && getBestTarget().getPoseAmbiguity() < .1) {
          target = m_layout.getTagPose(getBestTarget().getFiducialId()).get().toPose2d();
          distance = pose.getTranslation().getDistance(target.getTranslation());
          return distance;
        }
      }
    } catch (Exception e) {
      return 999;
    }
    return 999;
  }

  public Optional<PhotonTrackedTarget> getTag(int tagID) {
    for (PhotonTrackedTarget target : m_camera.getLatestResult().getTargets()) {
      if (target.getFiducialId() == tagID) {
        return Optional.of(target);
      }
    }

    return Optional.empty();
  }

  public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
    var estStdDevs = Constants.VisionConstants.kSingleTagStdDevs;
    var targets = m_camera.getLatestResult().getTargets();
    int numTags = 0;
    double avgDist = 0;
    for (var tar : targets) {
      var tagPose = m_estimator.getFieldTags().getTagPose(tar.getFiducialId());
      if (tagPose.isEmpty()) continue;
      numTags++;
      avgDist +=
          tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }
    if (numTags == 0) return estStdDevs;
    avgDist /= numTags;
    if (numTags > 1) estStdDevs = Constants.VisionConstants.kMultiTagStdDevs;
    if (numTags == 1 && avgDist > 3.8)
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 2.3));

    return estStdDevs;
  }

  public Optional<Double> getTagYaw(int tagID) {
    Optional<PhotonTrackedTarget> target = getTag(tagID);
    if (target.isEmpty()) return Optional.empty();

    double yaw = target.get().getYaw();
    yaw += m_offset.getRotation().toRotation2d().getDegrees();

    return Optional.of(yaw);
  }

  public boolean getTagDistance() {
    if (getBestTarget() == null) return false;

    Optional<PhotonTrackedTarget> target = getTag(getBestTarget().getFiducialId());

    Optional<Double> yaw = getTagYaw(getBestTarget().getFiducialId());
    if (yaw.isEmpty()) return false;

    double distance =
        PhotonUtils.calculateDistanceToTargetMeters(
            m_offset.getZ(),
            m_layout.getTagPose(getBestTarget().getFiducialId()).get().getZ(),
            m_offset.getRotation().getY(),
            target.get().getPitch());
    return distance < 4;
  }

  public Optional<Double> getTagDistance(int tagID) {
    Optional<PhotonTrackedTarget> target = getTag(tagID);

    Optional<Double> yaw = getTagYaw(tagID);
    if (yaw.isEmpty()) return Optional.empty();

    double distance =
        PhotonUtils.calculateDistanceToTargetMeters(
            m_offset.getZ(),
            m_layout.getTagPose(tagID).get().getZ(),
            m_offset.getRotation().getY(),
            target.get().getPitch());

    SmartDashboard.putNumber(m_camera.getName() + " camera distance", distance);
    return Optional.of(distance);
  }

  public Optional<EstimatedRobotPose> getCameraResult(Pose2d prevPose) {
    try {
      if (getBestTarget() != null) {
        if (getHasTarget() && getBestTarget() != null && getBestTarget().getPoseAmbiguity() < .1) {

          m_estimator.setReferencePose(prevPose);
          Optional<EstimatedRobotPose> pose = m_estimator.update();
          return pose;
        }
      }
      return Optional.empty();

    } catch (Exception e) {
      return Optional.empty();
    }
  }

  public PhotonTrackedTarget getBestTarget() {
    var result = m_camera.getLatestResult();
    if (!result.hasTargets()) return null;
    return result.getBestTarget();
  }

  public boolean getHasTarget() {
    var result = m_camera.getLatestResult();
    return result.hasTargets();
  }

  public PhotonTrackedTarget getTargetById(int id) {
    var result = m_camera.getLatestResult();

    for (var target : result.getTargets()) {
      if (target.getFiducialId() == id) return target;
    }

    return null;
  }

  @Override
  public void periodic() {

    SmartDashboard.putBoolean("hasTarget", getHasTarget());
  }
}
