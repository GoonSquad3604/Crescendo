package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.util.swerve.COTSTalonFXSwerveConstants;
import frc.util.swerve.SwerveModuleConstants;

public final class Constants {

  public static final class General {
    public static final double stickDeadband = .1; // drive stick deadband
    public static final String CANIVORE_CANBUS = "drivetrain";
  }

  public static final class Swerve {
    public static final int pigeonID = 0;

    public static final COTSTalonFXSwerveConstants
        chosenModule = // TODO: This must be tuned to specific robot
        COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(
                COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

    /* Drivetrain Constants */
    public static final double trackWidth =
        Units.inchesToMeters(23); // TODO: This must be tuned to specific robot
    public static final double wheelBase =
        Units.inchesToMeters(23); // TODO: This must be tuned to specific robot
    public static final double wheelCircumference = chosenModule.wheelCircumference;

    /* Swerve Kinematics
     * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Module Gear Ratios */
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    /* Motor Inverts */
    public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
    public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

    /* Angle Encoder Invert */
    public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

    /* Swerve Current Limiting */
    public static final int angleCurrentLimit = 25;
    public static final int angleCurrentThreshold = 40;
    public static final double angleCurrentThresholdTime = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveCurrentLimit = 35;
    public static final int driveCurrentThreshold = 60;
    public static final double driveCurrentThresholdTime = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    /* Angle Motor PID Values */
    public static final double angleKP = chosenModule.angleKP;
    public static final double angleKI = chosenModule.angleKI;
    public static final double angleKD = chosenModule.angleKD;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.12; // TODO: This must be tuned to specific robot
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    /* Drive Motor Characterization Values From SYSID */
    public static final double driveKS = 0.32; // TODO: This must be tuned to specific robot
    public static final double driveKV = 1.51;
    public static final double driveKA = 0.27;

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double maxSpeed = 4.5; // TODO: This must be tuned to specific robot

    /** Radians per Second */
    public static final double maxAngularVelocity =
        10.0; // TODO: This must be tuned to specific robot

    /* Neutral Modes */
    public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
    public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 1;
      public static final int angleMotorID = 3;
      public static final int canCoderID = 20;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-108.98);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 40;
      public static final int angleMotorID = 2;
      public static final int canCoderID = 21;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(79.98);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 18;
      public static final int angleMotorID = 16;
      public static final int canCoderID = 22;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-64.42);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 19;
      public static final int angleMotorID = 17;
      public static final int canCoderID = 23;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(289.51);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    public static final HolonomicPathFollowerConfig pathFollowerConfig =
        new HolonomicPathFollowerConfig(
            new PIDConstants(1.0, 0, 0), // Translation constants
            new PIDConstants(1.0, 0, 0), // Rotation constants
            maxSpeed,
            wheelBase / 2, // Drive base radius (distance from center to furthest module)
            new ReplanningConfig());
  }

  public static final class ShooterConstants {
    public static final int leftID = 14;
    public static final int rightID = 15;
    public static final int indexID = 4;
    public static final int angleID = 5;

    // RPMS
    public static final int leftShooterSpeakerRPM = 5000;
    public static final int rightShooterSpeakerRPM = 5000;

    public static final int leftShooterAmpRPM = 1000;
    public static final int rightShooterAmpRPM = 1000;

    public static final int leftShooterTrapRPM = 3000;
    public static final int rightShooterTrapRPM = 3000;

    public static final int indexTrapRPM = 6000;

    // Shooter Positions
    public static final double shooterSpeaker = 0;
    public static final double shooterAmp = 0;
    public static final double shooterTrap = 0;
    public static final double shooterHome = 35;
    public static final double shooterTravel = 0;

    public static final double shooterMax = 58.6;

    public static final double shooterOffset = .4505;

    // PIDS
    public static final double shooterkP = 0.0005;
    public static final double shooterkI = 0.0;
    public static final double shooterkD = 0.0;
    public static final double shooterkF = 0.00016;

    public static final double angleUpP = 0.0;
    public static final double angleDownP = 0.0;

    public static final double anglekI = 0.0;
    public static final double anglekD = 0.0;

    public static final double indexkP = 0.0005;
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
  }

  public static final class IntakeConstants {
    public static final int intakeID = 6;
    public static final int hingeID = 12;

    public static final double intakekFF = 0.0001;
    public static final double intakekP = 0.0005;
    public static final double intakekI = 0.0;
    public static final double intakekD = 0.0;

    public static final double hingekP = 5.0;
    public static final double hingekI = 0.0;
    public static final double hingekD = 0.0;

    public static final double hingeDown = .377;
    public static final double hingeUp = .7;

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
