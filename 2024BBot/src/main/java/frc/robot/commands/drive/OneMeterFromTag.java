// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneMeterFromTag extends PIDCommand {
  /** Creates a new AimPID. */
  private SwerveDrive m_Drive;

  private SwerveDrive m_controller;
  private static final double GOAL_RANGE_METERS = 1;

  // private double angleTo =vision.targetToRobotRotation().getRadians();
  public OneMeterFromTag(SwerveDrive swerve, XboxController driver, Vision vision) {
    super(
        // The controller that the command will use
        new PIDController(0.065, 0, 0),
        // This should return the measurement
        () -> vision.getDistance(),
        // This should return the setpoint (can also be a constant)
        () -> GOAL_RANGE_METERS,
        // This uses the output
        output -> {
          swerve.drive(new Translation2d(-output * .05, 0), 0, true, true);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(.3, 1);
    m_Drive = SwerveDrive.getInstance();
    addRequirements(m_Drive);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
