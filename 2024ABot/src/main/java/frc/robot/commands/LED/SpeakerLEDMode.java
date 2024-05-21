// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LED;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;

public class SpeakerLEDMode extends Command {
  /** Creates a new SpeakerLEDMode. */
  private Shooter shooter;

  private LED s_Led;
  private Timer timer;

  public SpeakerLEDMode(LED led) {
    // Use addRequirements() here to declare subsystem dependencies.
    timer = new Timer();
    shooter = Shooter.getInstance();
    s_Led = led;
    addRequirements(s_Led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Led.setColor(0, 0, 0);
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] rpms = shooter.getRPMS();
    if (rpms[0] < -5900 && rpms[1] > 5900) {
      s_Led.setColor(0, 0, 255);
    } else {
      if (timer.get() > .3) {
        s_Led.setColor(10, 10, 31);
        timer.reset();
      } else if (timer.get() <= .3) {
        s_Led.setColor(0, 0, 255);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
