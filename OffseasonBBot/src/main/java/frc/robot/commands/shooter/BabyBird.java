// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Shooter;

public class BabyBird extends Command {
  /** Creates a new BabyBird. */
  private Shooter s_Shooter;

  private Index s_Index;
  private Timer timer;
  private int count;
  private boolean triggered;
  private boolean ended;

  //Feeds note from the source
  public BabyBird() {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Shooter = Shooter.getInstance();
    s_Index = Index.getInstance();
    timer = new Timer();
    addRequirements(s_Index, s_Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    count = 0;
    timer.reset();
    triggered = false;
    s_Index.babyBirdIndex();

    s_Shooter.setShooterRPM(
        Constants.ShooterConstants.babyBirdLeftRPM, Constants.ShooterConstants.babyBirdRightRPM);
    s_Shooter.shooterToPos(Constants.ShooterConstants.babyBirdPos);

    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (s_Index.hasNote() && count == 0 && !triggered) {
      count++;
      triggered = true;
    }

    // first falling edge
    if (triggered && !s_Index.hasNote() && count == 1) triggered = false;

    // second rising edge
    if (s_Index.hasNote() && count == 1 && !triggered) {
      s_Shooter.setShooterRPM(0, 0);
      triggered = true;
      count++;
      // s_Flipper.setFlipperUp();
      s_Index.setIndexPower(0);
      ended = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() >= 6 || ended;
  }
}
