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
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drive.SwerveDefaultDrive;
import frc.robot.commands.shooter.FeedUntillSensor;
import frc.robot.commands.shooter.RepositionNote;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
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
  private final Climber s_Climber = Climber.getInstance();

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

    driverRightBumper.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
   // operator1.onTrue(new InstantCommand(() -> s_Intake.runIntake()));
    //operator1.onFalse(new InstantCommand(() -> s_Intake.stopIntake()));
    operator1.onTrue(new ParallelRaceGroup(new InstantCommand(() -> s_Intake.runIntake()),new SequentialCommandGroup( new FeedUntillSensor(), new RepositionNote())));
    operator1.onFalse(new ParallelCommandGroup(new InstantCommand(() -> s_Intake.stopIntake()), new InstantCommand(() -> s_Shooter.indexStop())));

    operator2.onTrue(new InstantCommand(() -> s_Intake.raiseHinge()));
    operator2.onFalse(new InstantCommand(() -> s_Intake.stopHinge()));

    operator3.onTrue(new InstantCommand(() -> s_Intake.lowerHinge()));
    operator3.onFalse(new InstantCommand(() -> s_Intake.stopHinge()));

    driverY.onTrue(new InstantCommand(() -> s_Climber.raiseLeftClimber()));
    driverY.onFalse(new InstantCommand(() -> s_Climber.stopLeftClimber()));

    driverX.onTrue(new InstantCommand(() -> s_Climber.raiseRightClimber()));
    driverX.onFalse(new InstantCommand(() -> s_Climber.stopRightClimber()));

    driverA.onTrue(new InstantCommand(() -> s_Climber.lowerRightClimber()));
    driverA.onFalse(new InstantCommand(() -> s_Climber.stopRightClimber()));

    driverB.onTrue(new InstantCommand(() -> s_Climber.lowerLeftClimber()));
    driverB.onFalse(new InstantCommand(() -> s_Climber.stopLeftClimber()));

    // operator4.onTrue(new SequentialCommandGroup(new FeedUntillSensor(), new RepositionNote()));
    operator4.onTrue(new InstantCommand(() -> s_Shooter.setShooterRPM()));
    operator4.onFalse(new InstantCommand(() -> s_Shooter.stopShooter()));

    operator5.onTrue(new InstantCommand(() -> s_Shooter.setIndexRPM()));
    operator5.onFalse(new InstantCommand(() -> s_Shooter.indexStop()));

    // operator7.onTrue(new InstantCommand(() -> s_Shooter.indexStop()));

    // operator8.toggleOnFalse(new InstantCommand(() -> s_Shooter.stop()));

    operator10.onTrue(new InstantCommand(() -> s_Shooter.raiseAngle()));
    operator10.onFalse(new InstantCommand(() -> s_Shooter.stopAngle()));

    operator11.onTrue(new InstantCommand(() -> s_Shooter.lowerAngle()));
    operator11.onFalse(new InstantCommand(() -> s_Shooter.stopAngle()));

    operator12.onTrue(new InstantCommand(() -> s_Intake.vomit()));
    operator12.onFalse(new InstantCommand(() -> s_Intake.stopIntake()));

    // driverA.onTrue(new InstantCommand(() -> s_Shooter.setPower(1)));
    // driverA.onFalse(new InstantCommand(() -> s_Shooter.stop()));
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
