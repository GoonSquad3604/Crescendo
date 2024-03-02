// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RepositionForAmp extends Command {
  Shooter s_Shooter;
  private Timer timer;

  public RepositionForAmp() {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Shooter = Shooter.getInstance();
    timer = new Timer();
    addRequirements(s_Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // if sensor is triggered, move shooter to steeper angle.
    if (s_Shooter.hasNote()) {
      s_Shooter.setIndexPower(.2);
    } else {
      s_Shooter.indexStop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // stop index and shooter
    s_Shooter.stopShooter();
    s_Shooter.indexStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() >= .4;
  }
}
