// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShooterTo extends InstantCommand {

  Shooter m_Shooter;
  double m_reference;

  public ShooterTo(double reference) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_Shooter = Shooter.getInstance();

    addRequirements(m_Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if (m_Shooter.getShooterAngleClicks() > m_reference) {
      m_Shooter.setUpP();
    } else {
      m_Shooter.setDownP();
    }
    m_Shooter.shooterTo(m_reference);
  }
}
