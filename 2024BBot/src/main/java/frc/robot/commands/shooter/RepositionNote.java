// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class RepositionNote extends Command {

  Shooter s_Shooter;

  /** Creates a new RepositionNote. */
  public RepositionNote() {
    s_Shooter = Shooter.getInstance();
    addRequirements(s_Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Shooter.setIndexPower(-0.1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Shooter.indexStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return s_Shooter.hasNote();
  }
}
