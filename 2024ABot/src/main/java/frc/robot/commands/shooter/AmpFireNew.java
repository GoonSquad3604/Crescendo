// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Flipper;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Shooter;

public class AmpFireNew extends Command {
  private Index s_Index;
  private Flipper s_Flipper;
  private Shooter s_Shooter;
  private Timer timer;
  boolean triggered;
  private double count;

  private double speed;

  public AmpFireNew() {
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
    count = 0;
    speed = .05;
    timer.reset();
    triggered = false;
    s_Index.setIndexPower(.14);
    // s_Shooter.setPower(speed);
    // s_Shooter.setPower(.05);
    s_Shooter.setShooterRPM(
        Constants.ShooterConstants.leftShooterAmpRPM,
        Constants.ShooterConstants.rightShooterAmpRPM);

    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (s_Flipper.flipperSensor() && count == 0 && !triggered) {
      count++;
      triggered = true;
      // s_Shooter.setShooterRPM(0, 0);
      // s_Shooter.setPower(.03);
      // s_Flipper.setFlipperUp();
    }
    if (triggered && !s_Flipper.flipperSensor() && count == 1)
      //  {
      //   s_Shooter.setShooterRPM(0, 0);
      //   s_Flipper.setFlipperUp();
      // }
      triggered = false;

    if (s_Flipper.flipperSensor() && count == 1 && !triggered) {
      s_Shooter.setShooterRPM(0, 0);
      triggered = true;
      count++;
      // s_Flipper.setFlipperUp();
    }
    if (!s_Flipper.flipperSensor() && count == 2 && triggered) {
      triggered = false;
    }
    if (s_Flipper.flipperSensor() && count == 2 && !triggered) {
      s_Flipper.setFlipperUp();
      count++;
      // triggered = true;
    }

    if (!s_Flipper.flipperSensor() && count == 3) {
      // s_Flipper.setFlipperDown();
      // s_Flipper.setFlipperUp();

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // stop index and shooter
    s_Shooter.stopShooter();
    s_Index.indexStop();
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // should finish when timer is like 1 sec.
    return timer.get() == 20;
  }
}
