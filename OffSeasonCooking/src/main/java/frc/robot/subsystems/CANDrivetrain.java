// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/* This class declares the subsystem for the robot drivetrain if controllers are connected via CAN. Make sure to go to
 * RobotContainer and uncomment the line declaring this subsystem and comment the line for PWMDrivetrain.
 *
 * The subsystem contains the objects for the hardware contained in the mechanism and handles low level logic
 * for control. Subsystems are a mechanism that, when used in conjuction with command "Requirements", ensure
 * that hardware is only being used by 1 command at a time.
 */
public class CANDrivetrain extends SubsystemBase {
  /*Class member variables. These variables represent things the class needs to keep track of and use between
  different method calls. */
  public static DifferentialDrive diffDrive;
  private static CANDrivetrain _instance;
  DifferentialDriveOdometry robotOdometry;

  static Pose2d robotPose;
  WPI_PigeonIMU pigeon;

  SparkAbsoluteEncoder leftEncoder;
  SparkAbsoluteEncoder rightEncoder;

  CANSparkMax leftFront = new CANSparkMax(Constants.DriveConstants.leftFrontID, MotorType.kBrushless);
  CANSparkMax leftRear = new CANSparkMax(Constants.DriveConstants.leftRearID, MotorType.kBrushless);
  CANSparkMax rightFront = new CANSparkMax(Constants.DriveConstants.rightFrontID, MotorType.kBrushless);
  CANSparkMax rightRear = new CANSparkMax(Constants.DriveConstants.rightRearID, MotorType.kBrushless);


  /*Constructor. This method is called when an instance of the class is created. This should generally be used to set up
   * member variables and perform any configuration or set up necessary on hardware.
   */
  public CANDrivetrain() {

    rightEncoder = rightFront.getAbsoluteEncoder();
    leftEncoder = leftFront.getAbsoluteEncoder();

    pigeon = new WPI_PigeonIMU(Constants.General.pigeonID);

    pigeon.reset();

    robotOdometry = new DifferentialDriveOdometry(
      pigeon.getRotation2d(),
        leftEncoder.getPosition(), rightEncoder.getPosition(),
          new Pose2d(1.27, 5.55, pigeon.getRotation2d()));


    // Set the rear motors to follow the front motors.
    leftRear.follow(leftFront);
    rightRear.follow(rightFront);

    leftFront.setIdleMode(IdleMode.kBrake);
    rightFront.setIdleMode(IdleMode.kBrake);

    // Invert the left side so both side drive forward with positive motor outputs
    leftFront.setInverted(true);
    rightFront.setInverted(false);

    // Put the front motors into the differential drive object. This will control all 4 motors with
    // the rears set to follow the fronts
    diffDrive = new DifferentialDrive(leftFront, rightFront);


    AutoBuilder.configureRamsete(
      this::getPose, // Robot pose supplier
      this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getCurrentSpeeds, // Current ChassisSpeeds supplier
      this::driveVolts, // Method that will drive the robot given ChassisSpeeds
      new ReplanningConfig(), // Default path replanning config. See the API for the options here
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this // Reference to this subsystem to set requirements
    );
  }

  public static CANDrivetrain getInstance() {
    if(_instance == null) {
      _instance = new CANDrivetrain();
    }
    return _instance;
  }

  public Pose2d getPose() {
    return robotPose;
  }

  private void resetPose(Pose2d pose2d) {
    robotOdometry.resetPosition(pigeon.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition(), robotPose);
  }

  private ChassisSpeeds getCurrentSpeeds() {
    // Creating my kinematics object: track width of 27 inches
    DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(Units.inchesToMeters(27.0));

    // Example differential drive wheel speeds: 2 meters per second
    // for the left side, 3 meters per second for the right side.
    var wheelSpeeds = new DifferentialDriveWheelSpeeds(0.001, 0.001);

    // Convert to chassis speeds.
    ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(wheelSpeeds);

    return chassisSpeeds;
  }

  private void driveVolts(ChassisSpeeds chassisSpeeds) {
    // Linear velocity
    double linearVelocity = chassisSpeeds.vxMetersPerSecond;

    // Angular velocity
    double angularVelocity = chassisSpeeds.omegaRadiansPerSecond;

    diffDrive.arcadeDrive(linearVelocity, angularVelocity);
  }

  /*Method to control the drivetrain using arcade drive. Arcade drive takes a speed in the X (forward/back) direction
   * and a rotation about the Z (turning the robot about it's center) and uses these to control the drivetrain motors */
  public void arcadeDrive(double speed, double rotation) {
    diffDrive.arcadeDrive(-speed, rotation);
  }

  @Override
  public void periodic() {
    var gyroAngle = pigeon.getRotation2d();

    robotPose = robotOdometry.update(gyroAngle,
    leftEncoder.getPosition(),
    rightEncoder.getPosition());
  }
}