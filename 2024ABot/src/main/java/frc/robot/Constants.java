package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class Constants {

  public static final class General {
    public static final double stickDeadband = .1; // drive stick deadband
    public static final String CANIVORE_CANBUS = "drivetrain";
  }

  public static final class ShooterConstants {
    public static final int leftID = 14;
    public static final int rightID = 15;
    public static final int indexID = 4;
    public static final int angleID = 5;

    // RPMS
    public static final int leftShooterSpeakerRPM = 4000;
    public static final int rightShooterSpeakerRPM = 6000;

    public static final int leftShooterAmpRPM = 300;
    public static final int rightShooterAmpRPM = 300;

    public static final int leftShooterTrapRPM = 3300;
    public static final int rightShooterTrapRPM = 4200;

    public static final int indexSpeakerRPM = 6000;
    public static final double indexSpeakerSpeed = 1;

    public static final int indexTrapRPM = 6000;
    public static final double indexTrapSpeed = 1;

    public static final int indexAmpRPM = 800;

    public static final double indexAmpSpeed = 1;

    // Shooter Positions
    public static final double shooterSpeaker = 60;
    public static final double shooterAmp = 67; // .61656273
    public static final double shooterAmpPos = .6165627;
    public static final double shooterTrap = 55.5; // .639
    public static final double shooterHome = 60;
    public static final double shooterTravel = 0;

    public static final double shooterMax = 58.6;

    public static final double shooterOffset = .4505;

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

    public static final double indexkP = 0.00025;
    public static final double indexkI = 0.0;
    public static final double indexkD = 0.0;
    public static final double indexkF = 0.00016;
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

    public static final double rightClimbedPos = 10;
    public static final double leftClimbedPos = -20;

    public static final double maxRightClimberHeight = 225;
    public static final double maxLeftClimberHeight = -218;

    public static final double minRightClimberHeight = 2;
    public static final double minLeftClimberHeight = -2;
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

    public static final double hingeDown = .381;
    public static final double hingeUp = .65;

    public static final int intakeRPM = 2000;
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
