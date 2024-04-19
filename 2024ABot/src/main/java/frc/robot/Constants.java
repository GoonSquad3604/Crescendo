package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class Constants {

  public static final class General {
    public static final double stickDeadband = .1; // drive stick deadband
    public static final String CANIVORE_CANBUS = "drivetrain";
  }

  public static final class ShooterConstants {
    // shooterTable

    public static final int leftID = 15;
    public static final int rightID = 14;
    public static final int angleID = 5;

    // RPMS
    public static final int leftShooterSpeakerRPM = -6000;
    public static final int rightShooterSpeakerRPM = 6000;

    public static final int leftShooterAmpRPM = -150;
    public static final int rightShooterAmpRPM = 150;

    public static final int babyBirdLeftRPM = 200;
    public static final int babyBirdRightRPM = -200;

    public static final int leftShooterTrapRPM = -1000; // -3300
    public static final int rightShooterTrapRPM = 2000; // 4200

    // Shooter Positions
    public static final double shooterSpeaker = 56; // 56
    public static final double shooterAmp =
        40; // 44 30 34  //ramp 50 degrees .15 power .73 ramp pos
    public static final double shooterTrap = 68;
    public static final double shooterHome = 65.5;
    public static final double babyBirdPos = 50;
    public static final double passingAngle = 45;
    public static final double shooterTravel = 8;

    public static final double shooterMax = 68.8; // 68.8 degrees

    // PIDS
    public static final double shooterkP = 0.0005;
    public static final double shooterkI = 0.0;
    public static final double shooterkD = 0.0;
    public static final double shooterkF = 0.00016;

    public static final double angleP = 5.0;

    public static final double angleUpP = 0.0;
    public static final double angleDownP = 0.0;

    public static final double anglekI = 0.0;
    public static final double anglekD = 0.0;
  }

  public static final class ClimberConstants {
    public static final int rightClimbID = 11;
    public static final int leftClimbID = 7;

    public static final int leftClimbkP = 0;
    public static final int leftClimbkI = 0;
    public static final int leftClimbkD = 0;

    public static final int rightClimbkP = 0;
    public static final int rightClimbkI = 0;
    public static final int rightClimbkD = 0;

    public static final double rightClimbedPosStable = 2;
    public static final double leftClimbedPosStable = 2;

    public static final double rightClimbedPosRightTaller = 63;
    public static final double leftClimbedPosRightTaller = 2;

    public static final double rightClimbedPosLeftTaller = 2;
    public static final double leftClimbedPosLeftTaller = 63;

    public static final double maxRightClimberHeight = 124;
    public static final double maxLeftClimberHeight = 124;

    public static final double minRightClimberHeight = 2;
    public static final double minLeftClimberHeight = 2;

    public static final double magicAngle = 8;
  }

  public static final class IntakeConstants {
    public static final int intakeID = 6;
    public static final int hingeID = 8;

    public static final double intakekFF = 0.0001;
    public static final double intakekP = 0.0005;
    public static final double intakekI = 0.0;
    public static final double intakekD = 0.0;

    public static final double hingekP = 5.0;
    public static final double hingekI = 0.0;
    public static final double hingekD = 0.0;

    public static final double hingeDown = .6292;
    public static final double hingeUp = .3697;
    public static final double hingeStart = .2752;

    public static final int intakeRPM = 2000;
  }

  public static final class IndexConstants {
    public static final int indexID = 4;

    public static final int indexTrapRPM = -6000;
    public static final double indexTrapSpeed = 1;

    public static final int indexAmpRPM = -800;

    public static final double indexAmpSpeed = .14;
    public static final int indexSpeakerRPM = -6000;
    public static final double indexSpeakerSpeed = 1;

    public static final double indexkP = 0.00025;
    public static final double indexkI = 0.0;
    public static final double indexkD = 0.0;
    public static final double indexkF = 0.00016;

    public static final double feedSpeed = .3;
    public static final double repositionSpeed = -.1;
    public static final double repositionSpeedAuto = -.2;
  }

  public static final class LEDConstants {
    public static final int IDLeft = 1;
    public static final int lengthLeft = 15;

    public static final int IDRight = 0;
    public static final int lengthRight = 30;
  }

  public static final class FlipperConstants {
    public static final int flipperID = 10;
    public static final double flipperDown = .973;
    public static final double flipperUp = .668; // .457
    public static final double crap = .26;
  }

  public static final class rAMPConstants {
    public static final int rAMPID = 10;
    public static final double rAMPUP = .35;
    public static final double almostUp = .4;
    public static final double almostDown = .5;

    public static final double rAMPDOWN = .73;
  }

  public static class VisionConstants {

    /**
     * Array of PhotonVision camera names. The values here match ROBOT_TO_CAMERA_TRANSFORMS for the
     * camera's location.
     */
    public static final String[] APRILTAG_CAMERA_NAMES = {"Left", "Right"};

    public static final Pose2d BLUE_AMP_DISTANCE_TARGET =
        new Pose2d(1.95, 6.82, new Rotation2d(-Math.PI));
    public static final Pose2d RED_AMP_DISTANCE_TARGET =
        new Pose2d(14.6, 6.82, new Rotation2d(Math.PI));
    public static final Pose2d BLUE_SPEAKER_DISTANCE_TARGET =
        new Pose2d(0.2, 5.52, new Rotation2d(-Math.PI));
    public static final Pose2d RED_SPEAKER_DISTANCE_TARGET =
        new Pose2d(16.53, 5.52, new Rotation2d(Math.PI));
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(6, 6, 10);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(.6, .6, 7);

    /**
     * Physical location of the apriltag cameras on the robot, relative to the center of the robot.
     * The values here math APRILTAG_CAMERA_NAMES for the camera's name.
     */
    public static final Transform3d LEFT_ROBOT_TO_CAMERA =
        new Transform3d(
            new Translation3d(-.2667, 0.24765, 0.56896), // 11.5 inches
            new Rotation3d(
                Math.toRadians(0),
                -Math.toRadians(65),
                Math.toRadians(120))); // pitch 46.9  roll55  yaw

    public static final Transform3d RIGHT_ROBOT_TO_CAMERA =
        new Transform3d(
            new Translation3d(-.2667, -0.24765, 0.56896), // 11.5 inches
            new Rotation3d(
                Math.toRadians(0),
                -Math.toRadians(65),
                Math.toRadians(-120))); // pitch 46.9  roll55  yaw

    public static final double FIELD_LENGTH = 16.54175;
    public static final double FIELD_WIDTH = 8.0137;

    /**
     * Minimum target ambiguity. Targets with higher ambiguity will be discarded. Not appliable when
     * multiple tags are in view in a single camera.
     */
    public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;
  }

  public static final
  class AutoConstants { // TODO: The below constants are used in the example auto, and must be tuned
    // to specific robot
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    /* Constraint for the motion profilied robot angle controller */
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}
