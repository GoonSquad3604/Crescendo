package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.SwerveDrive;
import frc.roAim.subsystems.Vision;
import java.util.function.BooleanSupplier;
import frc.robot.commands.drive.SwerveDefaultDrive;

public class Aim extends Command {

  Vision m_Vision;
  SwerveDrive m_Drive;
  private double direction;
  private double speed = 1.0;

  public Aim() {
    m_Vision = Vision.getInstance();
    m_Drive = SwerveDrive.getInstance();

    addRequirements(m_Drive, m_Vision);
  }

  @Override
  public void initialize() {
    if(m_Vision.getHasTarget()) {
      end(false);
    }    
    
    if(m_Vision.getTx() > 0) direction = 1.0;
    else direction = -1.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Drive.drive(new Translation2d(0,0), speed*direction, true, false, false);
    if(m_Vision.getTx() > 0) direction = 1.0;
    else direction = -1.0;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_Vision.getTx()) < 0.1;
  }
}
