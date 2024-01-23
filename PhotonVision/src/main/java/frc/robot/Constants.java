// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.util.swerve.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class General{
        public static final double deadband = .1; //drive stick deadband
        public static final String CANIVORE_CANBUS = "drivetrain";
    }

    public static final class Swerve{

        // /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(22.5); //TODO: Match this to robot
        public static final double wheelBase = Units.inchesToMeters(22.5); //TODO: Math this to robot

        public static final int pigeonID = 25;
        
        /* Swerve Kinematics 
        * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */

        public static final Translation2d frontLeftLocation = new Translation2d(wheelBase / 2.0, trackWidth / 2.0);
        public static final Translation2d frontRightLocation = new Translation2d(wheelBase / 2.0, -trackWidth / 2.0);
        public static final Translation2d backLeftLocation = new Translation2d(-wheelBase / 2.0, trackWidth / 2.0);
        public static final Translation2d backRightLocation = new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0);


         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            frontLeftLocation,
            frontRightLocation,
            backLeftLocation,
            backRightLocation);

        //Swerve Moudle constants

        public static final double angleGearRatio = ((150.0 / 7.0) / 1.0);
        public static final double driveGearRatio = (6.75 / 1.0); //MK4 L2 gearing

        public static final boolean driveMotorInvert = true;
        public static final boolean angleMotorInvert = true;
        public static final boolean canCoderInvert = false;

        public static final double wheelCircumference = Units.inchesToMeters(11.0);

        //Angle Motor PID
        public static final double angleKP = 0.3;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;
        public static final double angleKF = 0.0;

        /* Drive Motor PID Values */

        public static final double driveKP = 0.05171; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

               /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.083472 / 12); //TODO: This must be tuned to specific robot
        public static final double driveKV = (2.6579 / 12);
        public static final double driveKA = (0.088189 / 12);
        
        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        //Swerve Ramping
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class FrontLeft { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 18;
            public static final int angleMotorID = 16;
            public static final int canCoderID = 20;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(285.117);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class FrontRight { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 21;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(311.396);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class BackLeft { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 19;
            public static final int angleMotorID = 17;
            public static final int canCoderID = 22;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(193.886);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class BackRight { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 40;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 23;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(78.31);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

    }

    public static final class ArmConstants {
        public static final int shoulder1ID = 15;
        //meaning of life, the universe and everything
        public static final int shoulder2ID = 4;
        public static final int elbowID = 5;
        public static final int clawId = 14;

        // Arm lengths
        public static final double bottomArmLength = 5; // 5 is temporary
        public static final double topArmLength = 5; // 5 is temporary

        //Arm Origin Length. The Origin of the arm is located how far from the back bumbers and arm.
        public static final double armOriginXOffset = .3326;
        public static final double armOriginYOffset = 1.07;

        // Gear ratios
        public static final double gearRatio = 1;

        // Claw PID
        public static final double clawP = 0.04;
        public static final double clawI = 0;
        public static final double clawD = 0;

        // Shoulder PID
        public static final double shoulderUpP = 5.0;
        public static final double shoulderDownP = 5.0;
        public static final double shoulderI = 0;
        public static final double shoulderD = 0.00;

        // Elbow PID
        public static final double elbowUpP = 4.0;
        public static final double elbowDownP = 2.0;
        public static final double elbowI = 0;
        public static final double elbowD = 0;

        // Positions

        // Cone
        public static final double highConeShoulder = 325.246468;
        public static final double highConeElbow = 111.305462;

        public static final double midConeShoulder = 271.494505;
        public static final double midConeElbow = 8.0416982;

        // Cube
        public static final double highCubeShoulder = 321.013920;
        public static final double highCubeElbow = 136.178917;

        public static final double midCubeShoulder = 273.951023;
        public static final double midCubeElbow = 8.0416982;

        // Hybrid
        public static final double hybridShoulder = 258.141031;
        public static final double hybridElbow = 69.862753;

        // Claw
        public static double closedCone = -118.501411;
        public static double closedCube = -45.237606;




    }

    public static final class LEDConstants {
        public static final int led1 = 0;
        public static final int led2 = 1;
    }

    public static final class IntakeConstants {

        // Hinge
        public static final int leftHingeId = 13;
        public static final int rightHingeId = 7;
        public static final double hingeP = 0.1;
        public static final double hingeI = 0;
        public static final double hingeD = 0;
        public static final double hingeDown = -20;

        // Spaghetti
        public static final int intakeId = 6;

        // Intake speeds
        public static final double coneSpeed = 1.0;
        public static final double cubeSpeed = 0.25;

    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 0.0;
        public static final double kPYController = 0.0;
        //public static final double kIYController = 0.1;
        //public static final double kIXController = 0.1;
        public static final double kPThetaController = 0.4;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}
