// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

public class Aim extends Command {
  /** Creates a new Aim. */
  private double MaxSpeed =
      TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate =
      1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
  CommandSwerveDrivetrain m_Swerve;
  Vision m_Vision;
  private double direction;
  private double speed;
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric


  public Aim(CommandSwerveDrivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Swerve = drive;
    m_Vision = Vision.getInstance();
    addRequirements(m_Swerve, m_Vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(m_Vision.getHasTarget()){
      end(false);
    }
    if(m_Vision.getTx() > 0) direction = 1;
    else direction = - 1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Swerve.applyRequest(() -> drive.withVelocityX(0).withVelocityY(0).withRotationalRate(direction));
    if(m_Vision.getTx() > 0) direction = -.5;
    else direction = .5;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_Vision.getTx()) < .3;
  }
}
