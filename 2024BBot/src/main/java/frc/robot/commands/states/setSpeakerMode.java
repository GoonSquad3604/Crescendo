// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.StateController;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetSpeakerMode extends InstantCommand {
  Shooter s_Shooter;
  Intake s_Intake;
  StateController s_StateController;

  public SetSpeakerMode() {
    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    new InstantCommand(() -> s_StateController.setSpeaker(), s_StateController);
    new InstantCommand(() -> s_Intake.setHingeTo(Constants.IntakeConstants.hingeUp), s_Intake);
    new InstantCommand(() -> s_Shooter.shooterTo(Constants.ShooterConstants.shooterSpeaker));
  }
}
