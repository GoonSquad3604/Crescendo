package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

public class Aim extends Command {

  Vision m_Vision;
  private SwerveDrive m_Drive;
  private double direction;
  private double speed = 1.0;

  private double tx1;

  public Aim(SwerveDrive drive) {
    m_Vision = Vision.getInstance();
    m_Drive = drive;

    addRequirements(m_Drive, m_Vision);
  }

  @Override
  public void initialize() {
    if (m_Vision.getHasTarget()) {
      end(false);
    }
    if (m_Vision.getTx() > 0) direction = 1.0;
    else direction = -1.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Drive.drive(new Translation2d(0, 0), speed * direction, true, false);
    if (m_Vision.getTx() > 0) direction = -.5;
    else direction = .5;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_Vision.getTx()) < 0.3;
  }
}
