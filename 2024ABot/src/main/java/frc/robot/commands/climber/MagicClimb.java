// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class MagicClimb extends Command {
  /** Creates a new MagicClimb. */
  Climber m_climb;

  CommandSwerveDrivetrain m_drive;
  double pitch;
  boolean dontCorrect;

  private Timer timer;

  public MagicClimb(Climber climb, CommandSwerveDrivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    pitch = drive.getPigeon2().getPitch().getValueAsDouble();
    m_climb = climb;
    m_drive = drive;
    timer = new Timer();
    addRequirements(m_climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    dontCorrect = false;
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pitch = m_drive.getPigeon2().getPitch().getValueAsDouble();
    if (-8 < pitch && pitch < 8 && !dontCorrect) {
      m_climb.climberTo(
          Constants.ClimberConstants.leftClimbedPosStable,
          Constants.ClimberConstants.rightClimbedPosStable);
    } else if (pitch >= 8) {
      m_climb.climberTo(
          Constants.ClimberConstants.leftClimbedPosLeftTaller,
          Constants.ClimberConstants.rightClimbedPosLeftTaller);
      dontCorrect = true;
    } else if (pitch <= -8) {
      m_climb.climberTo(
          Constants.ClimberConstants.leftClimbedPosRightTaller,
          Constants.ClimberConstants.rightClimbedPosRightTaller);
      dontCorrect = true;
    }
    // System.out.println(m_drive.getPigeon2().getPitch().getValueAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climb.stopClimber();
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pitch > 20 || pitch < -20 || timer.get() >= 4;
  }
}
