// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;

public class AutoShootAngle extends Command {
  /** Creates a new AutoShootAngle. */

  CommandSwerveDrivetrain m_drive;
  Shooter m_shooter;
  double angle;

  public AutoShootAngle(CommandSwerveDrivetrain drive, Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_shooter = shooter;
    addRequirements(m_drive,m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

        Pose2d pose = m_drive.getState().Pose;
        
        Pose2d target = Constants.VisionConstants.SPEAKER_DISTANCE_TARGET;
        double distance = pose.getTranslation().getDistance(target.getTranslation());
        angle = Math.atan(1.524/(distance-0.2286));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //m_shooter.shooterTo(angle);
    System.out.println(Math.toDegrees(angle));
    SmartDashboard.putNumber("angleTo", angle);
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
