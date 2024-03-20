// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.StateController;

public class RepositionNoteAuto extends Command {
  Index s_Index;
  Shooter s_Shooter;
  Intake s_Intake;
  StateController s_StateController;

  /** Creates a new RepositionNote. */
  public RepositionNoteAuto() {
    s_Index = Index.getInstance();
    s_Intake = Intake.getInstance();
    s_StateController = StateController.getInstance();
    addRequirements(s_Intake, s_StateController, s_Index);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Index.setIndexPower(-0.1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

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
