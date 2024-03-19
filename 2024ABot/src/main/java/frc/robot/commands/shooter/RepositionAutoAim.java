// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.util.RobotMode;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RepositionAutoAim extends InstantCommand {
  Index s_Index;
  Shooter s_Shooter;
  Intake s_Intake;
  CommandSwerveDrivetrain s_Drive;
  double angleRAD;
  double angle;
  double distance;
  Pose2d target;
  public RepositionAutoAim(Index index, Intake intake, Shooter shooter, CommandSwerveDrivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Index = index;
    s_Intake = intake;
    s_Shooter = shooter;
    s_Drive = drive;
    addRequirements(s_Intake, s_Index, s_Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d pose = s_Drive.getState().Pose;
        target =Constants.VisionConstants.RED_SPEAKER_DISTANCE_TARGET;
          
        var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
             if(alliance.get() == DriverStation.Alliance.Blue)target = Constants.VisionConstants.BLUE_SPEAKER_DISTANCE_TARGET;
          }
         distance = pose.getTranslation().getDistance(target.getTranslation());
        angleRAD = Math.atan(1.524/(distance-0.2286));
    angle = Math.toDegrees(angleRAD);
    if(angle>56){
      s_Shooter.shooterTo(56);
    }
    if(angle<13){
      s_Shooter.shooterTo(13);
    }
    if( angle<56 && angle>13&& distance<3) {
            s_Shooter.shooterTo(angle);
      }
      if(angle<56 && angle>13&& distance>3) {
            s_Shooter.shooterTo(angle+distance);
      }
  }
}
