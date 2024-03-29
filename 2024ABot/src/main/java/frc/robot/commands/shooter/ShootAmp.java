// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flipper;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Shooter;

public class ShootAmp extends Command {
  private Index s_Index;
  private Flipper s_Flipper;
  private Shooter s_Shooter;
  private Timer timer;
  boolean started;

  private double speed;

  public ShootAmp() {
    s_Flipper = Flipper.getInstance();
    s_Shooter = Shooter.getInstance();
    s_Index = Index.getInstance();
    timer = new Timer();

    addRequirements(s_Shooter, s_Flipper, s_Index);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // set indexer to speed to fire
    // set shooter to amp fire rpm.
    // start timer
    speed = .01;
    timer.reset();
    started = false;
    s_Index.setIndexPower(.1);
    s_Shooter.setPower(speed);
    // s_Shooter.setPower(.05);
    // s_Shooter.setShooterRPM(
    //     Constants.ShooterConstants.leftShooterAmpRPM+100,
    //     Constants.ShooterConstants.rightShooterAmpRPM-100);

    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if sensor is triggered, move shooter to steeper angle.
    s_Shooter.setPower(speed);
    s_Index.setIndexPower(.2);
    if (s_Index.hasNote()) {
      started = true;
      // s_Flipper.setFlipperUp();
    }
    if (!s_Index.hasNote()) speed += .0006;
    // one second
    if (timer.get() >= 1 && started) {
      // s_Shooter.setShooterRPM(
      //   0,
      //   0);
      s_Flipper.setFlipperUp();
      // s_Shooter.setShooterRPM(
      //   Constants.ShooterConstants.leftShooterAmpRPM+100,
      //   Constants.ShooterConstants.rightShooterAmpRPM-100);
      s_Index.setIndexPower(.1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // stop index and shooter
    s_Shooter.stopShooter();
    s_Index.indexStop();
    started = false;
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // should finish when timer is like 1 sec.
    return timer.get() == 1.5;
  }
}
