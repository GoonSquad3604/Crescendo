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
public class AimPID extends PIDCommand {
  /** Creates a new Aim2PID. */
  SwerveDrive m_Drive;
  SwerveDrive m_controller;
  public AimPID(SwerveDrive swerve, XboxController driver, Vision vision) {
    super(
        // The controller that the command will use
        new PIDController(0.005, 0, 0.),
        // This should return the measurement
        () -> Math.abs(swerve.getPose().getRotation().getRadians()),
        // This should return the setpoint (can also be a constant)
        () -> vision.targetToRobotRotation(swerve),
        // This uses the output
        output -> {
            swerve.drive(new Translation2d(driver.getLeftY(),driver.getLeftX()),output, true,false);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(.1,1);
    getController().enableContinuousInput(-180,180);
    m_Drive = SwerveDrive.getInstance();
    addRequirements(m_Drive);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
