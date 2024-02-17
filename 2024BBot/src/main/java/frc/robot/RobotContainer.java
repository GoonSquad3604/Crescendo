// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.drive.SwerveDefaultDrive;
import frc.robot.commands.shooter.FeedUntillSensor;
import frc.robot.commands.shooter.RepositionNote;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.StateController;
import frc.robot.subsystems.SwerveDrive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private XboxController driver = new XboxController(0);
  private XboxController operatorController = new XboxController(1);
  private Joystick operatorJoystick = new Joystick(2);

  private final JoystickButton robotCentric =
      new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

  private final SendableChooser<Command> autoChooser;

  /* Subsystems */
  private final SwerveDrive s_Swerve = SwerveDrive.getInstance();
  private final Shooter s_Shooter = Shooter.getInstance();
  private final Intake s_Intake = Intake.getInstance();
 // private final Climber s_Climber = Climber.getInstance();
  private final StateController s_StateController = StateController.getInstance();

  public RobotContainer() {

    /* Default Commands */
    s_Swerve.setDefaultCommand(
        new SwerveDefaultDrive(
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX(),
            () -> driver.getLeftTriggerAxis(),
            robotCentric));

    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);
    
  }

  private void configureBindings() {
    // Driver

    JoystickButton driverX = new JoystickButton(driver, XboxController.Button.kX.value);
    JoystickButton driverB = new JoystickButton(driver, XboxController.Button.kB.value);
    JoystickButton driverY = new JoystickButton(driver, XboxController.Button.kY.value);
    JoystickButton driverA = new JoystickButton(driver, XboxController.Button.kA.value);
    JoystickButton driverRightBumper =
        new JoystickButton(driver, XboxController.Button.kRightBumper.value);

    // Operator button box
    JoystickButton operator1 = new JoystickButton(operatorJoystick, 1);
    JoystickButton operator2 = new JoystickButton(operatorJoystick, 2);
    JoystickButton operator3 = new JoystickButton(operatorJoystick, 3);
    JoystickButton operator4 = new JoystickButton(operatorJoystick, 4);
    JoystickButton operator5 = new JoystickButton(operatorJoystick, 5);
    JoystickButton operator6 = new JoystickButton(operatorJoystick, 6);
    JoystickButton operator7 = new JoystickButton(operatorJoystick, 7);
    JoystickButton operator8 = new JoystickButton(operatorJoystick, 8);
    JoystickButton operator9 = new JoystickButton(operatorJoystick, 9);
    JoystickButton operator10 = new JoystickButton(operatorJoystick, 10);
    JoystickButton operator11 = new JoystickButton(operatorJoystick, 11);
    JoystickButton operator12 = new JoystickButton(operatorJoystick, 12);

    Trigger homeTrigger = new Trigger(s_StateController::isHomeMode);
    Trigger indexTrigger = new Trigger(s_Shooter::hasNote);
    Trigger climberTrigger = new Trigger(s_StateController::isClimberMode);

    driverRightBumper.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
   operator1.onTrue(
        new ParallelCommandGroup(
            new InstantCommand(
                () -> s_Intake.setHingeTo(Constants.IntakeConstants.hingeDown), s_Intake),
            new SequentialCommandGroup(
              new InstantCommand(() -> s_StateController.setHome(), s_StateController),
              new InstantCommand(() -> s_Shooter.shooterTo(s_StateController.getAngle()),s_Shooter))));

    operator2.and(homeTrigger).onTrue(
        new ParallelCommandGroup(
            new InstantCommand(() -> s_Intake.runIntake()),
            new SequentialCommandGroup(new FeedUntillSensor(), new RepositionNote())));
    operator2.and(indexTrigger).onTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> s_Intake.stopIntake()),
                new InstantCommand(() -> s_Shooter.stopShooter())));
    operator2.onFalse(
        new ParallelCommandGroup(
            new InstantCommand(() -> s_Intake.stopIntake()),
            new InstantCommand(() -> s_Shooter.indexStop(), s_Shooter)));

    operator3.onTrue(new InstantCommand(() -> s_Intake.vomit()));
    operator3.onTrue(new InstantCommand(()-> s_Shooter.setIndexPower(-.3)));
    operator3.onFalse(new InstantCommand(() -> s_Intake.stopIntake()));
    operator3.onFalse(new InstantCommand(() -> s_Shooter.indexStop()));
    operator4.onTrue(
        new ParallelCommandGroup(
            new InstantCommand(() -> s_Intake.setHingeTo(Constants.IntakeConstants.hingeUp), s_Intake),
            new InstantCommand(() -> s_StateController.setTravel(), s_StateController)));

    operator5.onTrue(new ParallelCommandGroup(
      new InstantCommand(() -> s_StateController.setClimber(), s_StateController),
      new InstantCommand(() -> s_Intake.setHingeTo(Constants.IntakeConstants.hingeUp), s_Intake)));

 operator6.onTrue(new ParallelCommandGroup(
      new InstantCommand(() -> s_StateController.setSpeaker(), s_StateController),
      new InstantCommand(() -> s_Intake.setHingeTo(Constants.IntakeConstants.hingeUp), s_Intake))); 

  operator7.onTrue(new ParallelCommandGroup(
      new InstantCommand(() -> s_StateController.setAmp(), s_StateController),
      new InstantCommand(() -> s_Intake.setHingeTo(Constants.IntakeConstants.hingeUp), s_Intake)));
operator8.onTrue(new ParallelCommandGroup(
      new InstantCommand(() -> s_StateController.setTrap(), s_StateController),
      new InstantCommand(() -> s_Intake.setHingeTo(Constants.IntakeConstants.hingeUp), s_Intake)));

   // operator9.onTrue(new InstantCommand(() -> s_Shooter.setShooterRPM(s_StateController.getLeftShooterSpeed() ,s_StateController.getRightShooterSpeed()), s_Shooter));
   operator9.onTrue(new InstantCommand(() -> s_Shooter.setShooterRPMSpeaker()));
    operator9.and(climberTrigger).onTrue(
      new ParallelCommandGroup(
        new InstantCommand(() -> s_Shooter.stopShooter(), s_Shooter),
        new InstantCommand(() -> s_Shooter.indexStop())
    ));
    // operator9.onFalse(new InstantCommand(() -> s_Shooter.stopShooter()));
    operator10.onTrue(new InstantCommand(() -> s_Shooter.setIndexPower(s_StateController.getIndexSpeed())));
    operator10.and(climberTrigger).onTrue(
      new ParallelCommandGroup(
        new InstantCommand(() -> s_Shooter.stopShooter(), s_Shooter),
        new InstantCommand(() -> s_Shooter.indexStop())
    ));
    operator10.onFalse(
      new ParallelCommandGroup(
        new InstantCommand(() -> s_Shooter.stopShooter(), s_Shooter),
        new InstantCommand(() -> s_Shooter.indexStop())
    ));
    operator11.onTrue(new InstantCommand(() -> s_Shooter.shooterTo(s_StateController.getAngle()),s_Shooter));
    operator11.and(climberTrigger).onTrue(new InstantCommand(() -> s_Shooter.stopAngle(),s_Shooter));
    // operator11.and(climberTrigger).onTrue(new InstantCommand(() -> s_Climber.raiseCimber()));
    // operator12.and(climberTrigger).onTrue(new InstantCommand(() -> s_Climber.lowerClimber()));



    // operator1.onTrue(new InstantCommand(() -> s_Intake.runIntake()));
    // operator1.onFalse(new InstantCommand(() -> s_Intake.stopIntake()));

    // operator1.onTrue(new ParallelCommandGroup(new InstantCommand(() -> s_Intake.runIntake()),new
    // SequentialCommandGroup(new FeedUntillSensor(), new RepositionNote())));
    // operator1.onFalse(new ParallelCommandGroup(new InstantCommand(() -> s_Intake.stopIntake()),
    // new InstantCommand(() -> s_Shooter.indexStop(), s_Shooter)));

    // operator2.onTrue(new InstantCommand(() -> s_Intake.runIntake()));
    // operator2.onTrue(new SequentialCommandGroup(new FeedUntillSensor(), new RepositionNote()));
    // operator2.onFalse(new InstantCommand(() -> s_Intake.stopIntake()));
    // operator2.onFalse(new InstantCommand(() -> s_Shooter.indexStop(), s_Shooter));
    // operator2.onTrue(new InstantCommand(() -> s_Intake.raiseHinge()));
    // operator2.onFalse(new InstantCommand(() -> s_Intake.stopHinge()));

    //operator3.onTrue(new InstantCommand(() -> s_Intake.lowerHinge()));
    // operator3.onFalse(new InstantCommand(() -> s_Intake.stopHinge()));

    // driverY.onTrue(new InstantCommand(() -> s_Climber.climberUp()));
    // driverY.onFalse(new InstantCommand(() -> s_Climber.stopClimber()));

    // driverX.onTrue(new InstantCommand(() -> s_Climber.climberDown()));
    // driverX.onFalse(new InstantCommand(() -> s_Climber.stopClimber()));

    // operator3.onTrue(new InstantCommand(() -> s_Shooter.setIndexRPMLow(), s_Shooter));
    // operator3.onFalse(new InstantCommand(() -> s_Shooter.indexStop()));

    // operator4.onTrue(new SequentialCommandGroup(new FeedUntillSensor(), new RepositionNote()));
    // operator4.onTrue(new InstantCommand(() -> s_Shooter.setShooterRPMSpeaker()));
    // operator4.onFalse(new InstantCommand(() -> s_Shooter.stopShooter()));

    // operator6.onTrue(new InstantCommand(() -> s_Shooter.setShooterRPMAMP(), s_Shooter));
    // operator6.onFalse(new InstantCommand(() -> s_Shooter.stopShooter(), s_Shooter));

    // operator5.onTrue(new InstantCommand(() -> s_Shooter.setIndexPower(1), s_Shooter));
    // operator5.onFalse(new InstantCommand(() -> s_Shooter.indexStop()));

    // operator7.onTrue(new InstantCommand(() -> s_Shooter.indexStop()));

    //operator8.toggleOnFalse(new InstantCommand(() -> s_Shooter.stop()));

    // operator10.onTrue(new InstantCommand(() -> s_Shooter.raiseAngle()));
    // operator10.onFalse(new InstantCommand(() -> s_Shooter.stopAngle()));

    // operator11.onTrue(new InstantCommand(() -> s_Shooter.lowerAngle()));
    // operator11.onFalse(new InstantCommand(() -> s_Shooter.stopAngle()));

    // operator12.onTrue(new InstantCommand(() -> s_Shooter.shooterTo(50)));

    operator9.onTrue(new InstantCommand(() -> s_Shooter.shooterTo()));
    // operator8.onTrue(new InstantCommand(() -> s_Shooter.shooterTo(10)));
    
   driverA.onTrue(new InstantCommand(() -> s_Intake.cleam()));
   driverA.onFalse(new InstantCommand(() -> s_Intake.stopIntake()));

    //operator1.onTrue(new InstantCommand(() -> s_Intake.setHingeTo(Constants.IntakeConstants.hingeUp)));

    // operator3.onTrue(new InstantCommand(() -> s_Intake.lowerHinge()));
    // operator3.onFalse(new InstantCommand(() -> s_Intake.stopHinge()));
    // operator3.onTrue(new InstantCommand(() -> s_Intake.setHingeTo(Constants.IntakeConstants.hingeDown)));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
