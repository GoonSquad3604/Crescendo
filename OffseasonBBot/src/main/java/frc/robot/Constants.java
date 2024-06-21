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

//All of our values that don't change
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
    public static final double shooterSpeaker = 60;
    public static final double shooterHome = 60;
    public static final double babyBirdPos = 60;
    public static final double passingAngle = 30;
    public static final double shooterTravel = 10;
    public static final double trapAngle = 48.0;


    // public static final double shooterMax = 68.8; 

    // PIDS
    public static final double shooterkP = 0.0005;
    public static final double shooterkI = 0.0;
    public static final double shooterkD = 0.0;
    public static final double shooterkF = 0.00016;


    public static final double angleUpP = 0.0;
    public static final double angleDownP = 0.0;

    public static final double anglekP = 9.0;
    public static final double anglekI = 0.0;
    public static final double anglekD = 0.0;

  }

  public static final class ClimberConstants {
    public static final int rightClimbID = 11;
    public static final int leftClimbID = 7;

    public static final double leftClimbkP = Math.sqrt(0.25);
    public static final double leftClimbkI = 0;
    public static final double leftClimbkD = 0;

    public static final double rightClimbkP = 0.2 * Math.pow(1, 10);
    public static final double rightClimbkI = 0;
    public static final double rightClimbkD = 0;

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

    public static final double leftClimberUp = 0.4 * Math.sin(Math.PI/2);
    public static final double rightClimberUp = 0.4 * Math.cos(0);
    public static final double leftClimberDown = leftClimberUp * -1;
    public static final double rightClimberDown = leftClimberDown;

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

    public static final double hingeDown = .5350;
    public static final double hingeUp = .2965;

    public static final int intakeRPM = 2000;

    public static final double feedUntilSensorPower = (((1.5 * 2) / 10));;
    public static final double repositionForAmpPower = (double) 2 / 10;
    public static final double repositionNotePower = -1 * Math.abs(-0.1);
    public static final double repositionNoteAutoPower = -2 * Math.pow(10, -1);

    public static final double cleam = -0.5;
    public static final double raiseHinge = -0.3;
    public static final double lowerHinge = 0.3;
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

    public static final double babyBirdIndex = Math.max(0.2, 0.01);
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
