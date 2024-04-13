// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LED;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LED;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetLEDSPurple extends InstantCommand {
  private LED s_Led;

  public SetLEDSPurple(LED led) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Led = led;

    addRequirements(s_Led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Led.setColor(255, 0, 255);
  }
}
