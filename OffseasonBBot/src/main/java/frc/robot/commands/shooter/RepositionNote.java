// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.StateController;

//Indexes note until it is seen by the sensor. Then, moves shooter to travel mode position.
public class RepositionNote extends Command {

  Shooter s_Shooter;
  Intake s_Intake;
  Index s_Index;
  StateController s_StateController;

  /** Creates a new RepositionNote. */
  public RepositionNote() {
    s_Intake = Intake.getInstance();
    s_Shooter = Shooter.getInstance();
    s_Index = Index.getInstance();
    s_StateController = StateController.getInstance();
    addRequirements(s_Shooter, s_Intake, s_StateController, s_Index);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Index.setIndexPower(Constants.IntakeConstants.repositionNotePower);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Index.indexStop();

    s_Intake.setHingeTo(Constants.IntakeConstants.hingeUp);
    s_StateController.setTravel();
    s_Shooter.shooterToAngle(Constants.ShooterConstants.shooterHome);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return s_Index.hasNote();
  }
}
