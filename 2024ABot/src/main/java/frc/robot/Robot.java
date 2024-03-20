package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.*;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }
    // Set a metadata value

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    } else {
      setUseTiming(false); // Run as fast as possible
      String logPath =
          LogFileUtil
              .findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      Logger.addDataReceiver(
          new WPILOGWriter(
              LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in the
    // "Understanding Data Flow" page
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values
    // be added.
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    // if(m_robotContainer.getBestPose()!=null) {
    //   Pose2d x = (Pose2d)m_robotContainer.getBestPose();
    //   m_robotContainer.drivetrain.addVisionMeasurement(x,Timer.getFPGATimestamp());
    // }

    // if (m_Vision.robotPose()!=null) {
    //
    // m_robotContainer.drivetrain.addVisionMeasurement(m_Vision.robotPose(),Timer.getFPGATimestamp());
    // }
    // SmartDashboard.putNumber("train", defaultPeriodSecs);
    // vision get pose
    // if pose present
    // convert to pose
    // robotcontain.drivetrain.addvision
    // m_robotContainer.getAngle();
    var visionEst = m_robotContainer.getBestPose();
    visionEst.ifPresent(
        est -> {
          var estPose = est.estimatedPose.toPose2d();

          var estStdDevs = m_robotContainer.getEstimationStdDevs(estPose);

          m_robotContainer.drivetrain.addVisionMeasurement(
              est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
        });
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.clear();
  }

  @Override
  public void teleopPeriodic() {
    // var visionEst = m_Vision.getEstimatedGlobalPose();
    // visionEst.ifPresent(
    //   est-> {
    //       var estPose = est.estimatedPose.toPose2d();

    //       var estStdDevs = m_Vision.getEstimationStdDevs(estPose);

    //       m_robotContainer.drivetrain.addVisionMeasurement(est.estimatedPose.toPose2d(),
    // Timer.getFPGATimestamp(), estStdDevs);

    //   });
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
