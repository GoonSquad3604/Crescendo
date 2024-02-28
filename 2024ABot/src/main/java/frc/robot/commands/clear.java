// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class clear extends InstantCommand {
  Intake m_Intake;
  Shooter m_Shooter;
  public clear() {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Intake = Intake.getInstance();
    m_Shooter = Shooter.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Intake.stopIntake();
    m_Shooter.stopShooter();
    m_Shooter.indexStop();
  }
}
