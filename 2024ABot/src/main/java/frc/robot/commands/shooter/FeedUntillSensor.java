// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;

public class FeedUntillSensor extends Command {
  /** Creates a new FeedUntillSensor. */
  Index s_Index;
  private LED s_led;
  private Intake s_Intake;

  public FeedUntillSensor(LED left, Intake intake) {
    s_Index = Index.getInstance();
    s_led = left;
    s_Intake = intake;

    addRequirements(s_Index, s_led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Index.setIndexPower(.3); //.3
    s_led.setColor(255, 0, 0);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(s_Intake.hasNote()){
      s_led.setColor(0, 255, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Index.indexStop();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return s_Index.hasNote();
  }
}
