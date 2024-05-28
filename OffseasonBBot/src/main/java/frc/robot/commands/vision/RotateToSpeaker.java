// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

//Rotates robot until it faces the speaker, using AprilTags and the gyro
public class RotateToSpeaker extends ProfiledPIDCommand {
  private static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
  private final CommandSwerveDrivetrain drivetrain;
  Field2d field = new Field2d();
  Field2d bruh = new Field2d();
  // public static final Pose2d SPEAKER_DISTANCE_TARGET = new Pose2d(0.2, 5.52, new
  // Rotation2d(Math.PI));

  private static Rotation2d target = new Rotation2d();

  /** Creates a new RotateToSpeaker. */
  public RotateToSpeaker(CommandSwerveDrivetrain drivetrain) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            9,
            0,
            0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(6, 6)),
        // This should return the measurement
        () -> drivetrain.getState().Pose.getRotation().getRadians(),
        // This should return the goal (can also be a constant)
        () -> new TrapezoidProfile.State(target.getRadians(), 0.),
        // This uses the output
        (output, setpoint) -> {
          // Use the output (and setpoint, if desired) here
          drivetrain.setControl(drive.withRotationalRate(output + setpoint.velocity));
          SmartDashboard.putNumber("setpoint", setpoint.velocity);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(drivetrain);
    getController().enableContinuousInput(-Math.PI, Math.PI);

    this.drivetrain = drivetrain;
  }

  @Override
  public void initialize() {
    super.initialize();
    Pose2d target = Constants.VisionConstants.RED_SPEAKER_DISTANCE_TARGET;
    ;
    Pose2d pose = drivetrain.getState().Pose;

    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == DriverStation.Alliance.Blue)
        target = Constants.VisionConstants.BLUE_SPEAKER_DISTANCE_TARGET;
    }
    Translation2d distance =
        new Translation2d(pose.getX() - target.getX(), pose.getY() - target.getY());

    Rotation2d angle = distance.getAngle();
    RotateToSpeaker.target = angle;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(target.getRadians() - drivetrain.getState().Pose.getRotation().getRadians())
        < 0.1;
  }
}
