// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.StateController;
import org.littletonrobotics.junction.AutoLogOutput;

public class LookUpTableAuton extends Command {
  /** Creates a new AutoShootAngleNew. */
  Shooter m_shoot;

  @AutoLogOutput Pose2d pose;
  StateController stateController;
  CommandSwerveDrivetrain swerve;
  @AutoLogOutput double angle;

  public LookUpTableAuton(Shooter shoot, CommandSwerveDrivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerve = drive;
    m_shoot = shoot;

    stateController = StateController.getInstance();
    addRequirements(m_shoot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pose = swerve.getState().Pose;

    angle = m_shoot.lookUpTable(pose);
    m_shoot.shooterTo(angle);
  }

  @AutoLogOutput
  public double updateAngle() {
    return m_shoot.lookUpTable(pose);
  }

  @AutoLogOutput
  public Pose2d updatePose() {
    return swerve.getState().Pose;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
