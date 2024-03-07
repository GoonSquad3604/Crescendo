// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.stateController;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.commands.shooter.RepositionForAmp;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.StateController;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AmpMode extends InstantCommand {
  StateController m_StateController;
  Intake m_Intake;
  Shooter m_Shooter;
  Index m_Index;

  public AmpMode() {
    m_StateController = StateController.getInstance();
    m_Intake = Intake.getInstance();
    m_Shooter = Shooter.getInstance();
    m_Index = Index.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Intake, m_Shooter, m_StateController);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_StateController.setAmp();
    new RepositionForAmp();
    m_Intake.setHingeTo(Constants.IntakeConstants.hingeUp);
    m_Shooter.shooterTo(Constants.ShooterConstants.shooterAmp);
    // m_Shooter.setShooterRPM(Constants.ShooterConstants.leftShooterAmpRPM+200, Constants.ShooterConstants.rightShooterAmpRPM-200);
    m_Shooter.setPower(.07);

  }
}
