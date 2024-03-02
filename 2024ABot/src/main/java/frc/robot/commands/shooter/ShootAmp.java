// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Flipper;
import frc.robot.subsystems.Shooter;

public class ShootAmp extends Command {
  private Flipper s_Flipper;
  private Shooter s_Shooter;
  private Timer timer;

  /* For Ahmed:
      This is a skeleton of a command I want you to write to attempt to hit the amp.
      This command will tied to the fire button and be used only when in amp mode.

      The command should work like this:
      When started it will set the index as if it is firing
      Then When the sensor is hit, the shooter angle should move up to some designated position
      The goal would be to almost flick the note as we fire it, so that the bottom pops foward.

      Try starting the command when the shooter is around 55 to 60 degrees and set it to go as far up as it can during the fire.

      Below I put some comments setting up a skeleton of it.
      Please finish the command
  */

  public ShootAmp() {
    s_Flipper = Flipper.getInstance();
    s_Shooter = Shooter.getInstance();
    timer = new Timer();
    addRequirements(s_Shooter, s_Flipper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // set indexer to speed to fire
    // set shooter to amp fire rpm.
    // start timer
    s_Shooter.setIndexPower(.1);
    s_Shooter.setShooterRPM(
        Constants.ShooterConstants.leftShooterAmpRPM,
        Constants.ShooterConstants.rightShooterAmpRPM);

    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if sensor is triggered, move shooter to steeper angle.
    if (s_Shooter.hasNote()) {
      s_Shooter.setIndexPower(.2);
      s_Shooter.setShooterRPM(0, 0);
      timer.delay(.5);
      s_Flipper.setFlipperUp();

      s_Shooter.setShooterRPM(
          Constants.ShooterConstants.leftShooterAmpRPM,
          Constants.ShooterConstants.rightShooterAmpRPM);
      s_Shooter.setIndexPower(.1);
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
    // should finish when timer is like 1 sec.
    return timer.get() == 1;
  }
}
