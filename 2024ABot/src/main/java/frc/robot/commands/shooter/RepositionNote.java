// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import com.pathplanner.lib.path.PathPlannerTrajectory.State;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.stateController.TravelMode;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.StateController;

public class RepositionNote extends Command {

  Shooter s_Shooter;
  Intake s_Intake;
  StateController s_StateController;

  /** Creates a new RepositionNote. */
  public RepositionNote() {
    s_Intake = Intake.getInstance();
    s_Shooter = Shooter.getInstance();
    s_StateController = StateController.getInstance();
    addRequirements(s_Shooter, s_Intake, s_StateController);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Shooter.setIndexPower(-0.1);
    s_Shooter.setShooterRPM(100, -100);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Shooter.indexStop();
    
    s_Intake.setHingeTo(Constants.IntakeConstants.hingeUp);
    s_StateController.setTravel();
    s_Shooter.shooterTo(s_StateController.getAngle());
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return s_Shooter.hasNote();
  }
}
