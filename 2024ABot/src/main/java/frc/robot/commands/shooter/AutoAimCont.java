// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.StateController;

public class AutoAimCont extends Command {
  /** Creates a new AutoShootAngle. */
  CommandSwerveDrivetrain m_drive;

  Shooter m_shooter;
  StateController m_statecontroller;
  double angleRAD;
  double angle;
  double distance;
  Pose2d target;

  public AutoAimCont(CommandSwerveDrivetrain drive, Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_shooter = shooter;
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d pose = m_drive.getState().Pose;
    target = Constants.VisionConstants.RED_SPEAKER_DISTANCE_TARGET;

    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == DriverStation.Alliance.Blue)
        target = Constants.VisionConstants.BLUE_SPEAKER_DISTANCE_TARGET;
    }
    distance = pose.getTranslation().getDistance(target.getTranslation());
    angleRAD = Math.atan(1.524 / (distance - 0.2286));
    angle = Math.toDegrees(angleRAD);
    if (angle > 56) {
      m_shooter.shooterTo(56);
    }
    if (angle < 13) {
      m_shooter.shooterTo(45);
    }
    if (angle < 56 && angle > 13 && distance < 3.5) {
      m_shooter.shooterTo(angle + 6);
    }
    if (angle < 56 && angle > 13 && distance > 3.5) {
      m_shooter.shooterTo(angle + distance * 1.2);
    }
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
