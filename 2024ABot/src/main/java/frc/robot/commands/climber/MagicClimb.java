// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class MagicClimb extends Command {
  /** Creates a new MagicClimb. */
  Climber m_climb;
  CommandSwerveDrivetrain m_drive;
  double roll;
  boolean dontCorrect;
  public MagicClimb(Climber climb, CommandSwerveDrivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    roll = drive.getPigeon2().getRoll().getValueAsDouble();
    m_climb = climb;
    m_drive = drive;
    addRequirements(m_climb, m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dontCorrect = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   roll = m_drive.getPigeon2().getRoll().getValueAsDouble();
    if(-5<roll && roll<5 && !dontCorrect) {
      m_climb.climberTo(Constants.ClimberConstants.leftClimbedPosStable, Constants.ClimberConstants.rightClimbedPosStable);
    } else if(roll >= 5) {
      m_climb.climberTo(Constants.ClimberConstants.leftClimbedPosLeftTaller, Constants.ClimberConstants.rightClimbedPosLeftTaller);
      dontCorrect = true;
    }
    else if(roll <= -5) {
      m_climb.climberTo(Constants.ClimberConstants.leftClimbedPosRightTaller, Constants.ClimberConstants.rightClimbedPosRightTaller);
      dontCorrect = true;
    }
    System.out.println(m_drive.getPigeon2().getRoll().getValue());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climb.stopClimber();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return roll>20 ||roll<-20;
  }
}
