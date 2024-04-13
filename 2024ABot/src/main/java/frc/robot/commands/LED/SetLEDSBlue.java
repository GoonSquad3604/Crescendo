// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LED;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LED;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetLEDSBlue extends InstantCommand {
  private LED s_Left;
  private LED s_Right;

  public SetLEDSBlue(LED left, LED right) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Left = left;
    s_Right = right;

    addRequirements(s_Left, s_Right);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Left.setColor(0, 0, 255);
    s_Right.setColor(0, 0, 255);
  }
}
