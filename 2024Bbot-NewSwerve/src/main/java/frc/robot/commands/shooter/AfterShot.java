// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.StateController;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AfterShot extends InstantCommand {
  Shooter m_Shooter;
  StateController m_StateContoller;
  private Timer timer;

  
  public AfterShot() {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Shooter = Shooter.getInstance();
    m_StateContoller = StateController.getInstance();
    timer = new Timer();

    addRequirements(m_Shooter, m_StateContoller);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.delay(1);
    m_StateContoller.setTravel();
    m_Shooter.shooterTo(m_StateContoller.getAngle());
  }
}
