// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
Joystick operatorJoystick = new Joystick(2);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

configureBindings();

Intake.getInstance();

JoystickButton operator1 = new JoystickButton(operatorJoystick, 1);
JoystickButton operator2 = new JoystickButton(operatorJoystick, 2);
JoystickButton operator3 = new JoystickButton(operatorJoystick, 3);
JoystickButton operator4 = new JoystickButton(operatorJoystick, 4);

  operator1.onTrue(new InstantCommand(() -> Intake.runIntake()));
  operator1.onFalse(new InstantCommand(() -> Intake.stopIntake()));
  operator2.onTrue(new InstantCommand(() -> Intake.vomit()));
  operator2.onFalse(new InstantCommand(() -> Intake.stopIntake()));
  operator3.onTrue(new InstantCommand(() -> Intake.hingePosition(Constants.HingeConstants.hingeDown)));
   operator3.onTrue(new InstantCommand(() -> Intake.raiseHinge()));
   operator3.onFalse(new InstantCommand(() -> Intake.stopHinge()));
   operator4.onTrue(new InstantCommand(() -> Intake.lowerHinge()));
   operator4.onFalse(new InstantCommand(() -> Intake.stopHinge()));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
   JoystickButton operator1 = new JoystickButton(operatorJoystick, 1);
JoystickButton operator2 = new JoystickButton(operatorJoystick, 2);



 operator1.onTrue(new InstantCommand(() -> Intake.runIntake()));
 operator1.onFalse(new InstantCommand(() -> Intake.stopIntake()));
 operator2.onTrue(new InstantCommand(() -> Intake.vomit()));
  operator2.onFalse(new InstantCommand(() -> Intake.stopIntake()));
 

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
   
  }

  public Command getAutonomousCommand() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getAutonomousCommand'");
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
    // An example command will be run in autonomous
    
  }

