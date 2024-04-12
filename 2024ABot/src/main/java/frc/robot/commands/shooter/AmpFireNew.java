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
    count = 0;
    timer.reset();
    triggered = false;
    s_Index.setIndexPower(Constants.IndexConstants.indexAmpSpeed);

    s_Shooter.setShooterRPM(
        Constants.ShooterConstants.leftShooterAmpRPM,
        Constants.ShooterConstants.rightShooterAmpRPM);

    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //first rising edge
    if (s_Flipper.flipperSensor() && count == 0 && !triggered) {
      count++;
      triggered = true;
    }

    //first falling edge
    if (triggered && !s_Flipper.flipperSensor() && count == 1)
      triggered = false;

    //second rising edge
    if (s_Flipper.flipperSensor() && count == 1 && !triggered) {
      s_Shooter.setShooterRPM(0, 0);
      triggered = true;
      count++;
      // s_Flipper.setFlipperUp();
    }

    // second falling edge;
    if (!s_Flipper.flipperSensor() && count == 2 && triggered) {
      triggered = false;
    }

    //third rising edge;
    if (s_Flipper.flipperSensor() && count == 2 && !triggered) {
      s_Flipper.setFlipperUp();
      count++;
      // triggered = true;
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
    return timer.get() >= 3;
  }
}
