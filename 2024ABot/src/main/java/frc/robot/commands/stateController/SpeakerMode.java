// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.stateController;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.StateController;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SpeakerMode extends InstantCommand {
  Intake m_Intake;
  Shooter m_Shooter;
  StateController m_StateController;

  public SpeakerMode() {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Intake = Intake.getInstance();
    m_Shooter = Shooter.getInstance();
    m_StateController = StateController.getInstance();

    addRequirements(m_Intake, m_Shooter, m_StateController);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Intake.setHingeTo(Constants.IntakeConstants.hingeUp);
    m_StateController.setSpeaker();
    // m_Shooter.shooterTo(40);
    m_Shooter.setShooterRPM(
        m_StateController.getLeftShooterSpeed(), m_StateController.getRightShooterSpeed());
  }
}
