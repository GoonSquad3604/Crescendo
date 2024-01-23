// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

public class AimAtTag extends Command {
  /** Creates a new AimAtTag. */
   Vision m_Vision;
  SwerveDrive m_Drive;
  int tagIDs;
  Timer timer;
  public AimAtTag(int tagID) {
   m_Vision = Vision.getInstance();
   m_Drive = SwerveDrive.getInstance();
    tagIDs = tagID;
    addRequirements(m_Drive, m_Vision);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
