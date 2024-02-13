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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drive.SwerveDefaultDrive;
import frc.robot.subsystems.Intake;
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

  // private final SendableChooser<Command> autoChooser;

  /* Subsystems */
  private final SwerveDrive s_Swerve = SwerveDrive.getInstance();
  private final Intake s_Intake = Intake.getInstance();
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

    // autoChooser = AutoBuilder.buildAutoChooser();
    // SmartDashboard.putData("Auto Mode", autoChooser);
  }

  private void configureBindings() {
    // Driver

    JoystickButton driverX = new JoystickButton(driver, XboxController.Button.kX.value);
    JoystickButton driverB = new JoystickButton(driver, XboxController.Button.kB.value);
    JoystickButton driverY = new JoystickButton(driver, XboxController.Button.kY.value);
    JoystickButton driverA = new JoystickButton(driver, XboxController.Button.kA.value);
    JoystickButton driverRightBumper =
        new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    JoystickButton driverHinge = new JoystickButton(driver, XboxController.Button.kStart.value);

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

    driverA.onTrue(new InstantCommand(()-> s_Intake.runIntake()));
    driverA.onFalse(new InstantCommand(()-> s_Intake.stopIntake()));

    driverB.onTrue(new InstantCommand(()-> s_Intake.raiseHinge()));
    driverB.onFalse(new InstantCommand(()-> s_Intake.stopHinge()));

    driverY.onTrue(new InstantCommand(()-> s_Intake.lowerHinge()));
    driverY.onFalse(new InstantCommand(()-> s_Intake.stopHinge()));

    //intake up
    driverHinge.onTrue(new InstantCommand(() -> s_Intake.setHingeTo(.7)));

    //intake down
    driverX.onTrue(new InstantCommand(() -> s_Intake.setHingeTo(.39)));


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
