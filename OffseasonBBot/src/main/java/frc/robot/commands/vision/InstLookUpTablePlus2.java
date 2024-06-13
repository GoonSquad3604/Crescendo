// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.StateController;
import org.littletonrobotics.junction.AutoLogOutput;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

//Moves shooter to certain position based on the robots pose and our lookup table 
//The shooter moves when the command is initially scheduled
public class InstLookUpTablePlus2 extends InstantCommand {
  Shooter m_shoot;

  @AutoLogOutput Pose2d pose;
  StateController stateController;
  CommandSwerveDrivetrain swerve;
  @AutoLogOutput double angle;

  public InstLookUpTablePlus2(Shooter shoot, CommandSwerveDrivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerve = drive;
    m_shoot = shoot;

    stateController = StateController.getInstance();
    addRequirements(m_shoot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pose = swerve.getState().Pose;

    angle = m_shoot.lookUpTable(pose);
    m_shoot.shooterTo(angle + 2);
  }
}
