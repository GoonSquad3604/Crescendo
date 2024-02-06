// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
 //import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.subsystems.Climber;

public class RobotContainer {

  //Declare controllers
   //private XboxController driver = new XboxController(0);
  private Joystick operatorJoystick = new Joystick(2);


  //Declare subsystems
  private Climber s_Climber = Climber.getInstance();


  public RobotContainer() {
    configureBindings();
  }



  private void configureBindings() {

    /* Initializations */


    // Driver
     // JoystickButton driverX = new JoystickButton(driver, XboxController.Button.kX.value);
     // JoystickButton driverY = new JoystickButton(driver, XboxController.Button.kY.value);
     // JoystickButton driverA = new JoystickButton(driver, XboxController.Button.kA.value);
     // JoystickButton driverB = new JoystickButton(driver, XboxController.Button.kB.value);


    // Operator
    JoystickButton operator1 = new JoystickButton(operatorJoystick, 1);
    JoystickButton operator2 = new JoystickButton(operatorJoystick, 2);
    JoystickButton operator3 = new JoystickButton(operatorJoystick, 3);
    JoystickButton operator4 = new JoystickButton(operatorJoystick, 4);




    // Climber
     // driverX.onTrue(new InstantCommand(() -> s_Climber.raiseRightClimber()));
     // driverX.onFalse(new InstantCommand(() -> s_Climber.stopRightClimber()));

     // driverY.onTrue(new InstantCommand(() -> s_Climber.lowerRightClimber()));
     // driverY.onFalse(new InstantCommand(() -> s_Climber.stopRightClimber()));

     // driverA.onTrue(new InstantCommand(() -> s_Climber.lowerLeftClimber()));
     // driverA.onFalse(new InstantCommand(() -> s_Climber.stopLeftClimber()));

     // driverB.onTrue(new InstantCommand(() -> s_Climber.raiseLeftClimber()));
     // driverB.onFalse(new InstantCommand(() -> s_Climber.stopLeftClimber()));

    operator1.onTrue(new InstantCommand(() -> s_Climber.raiseRightClimber()));
    operator1.onFalse(new InstantCommand(() -> s_Climber.stopRightClimber()));

    operator2.onTrue(new InstantCommand(() -> s_Climber.lowerRightClimber()));
    operator2.onFalse(new InstantCommand(() -> s_Climber.stopRightClimber()));

    operator3.onTrue(new InstantCommand(() -> s_Climber.raiseLeftClimber()));
    operator3.onFalse(new InstantCommand(() -> s_Climber.stopLeftClimber()));

    operator4.onTrue(new InstantCommand(() -> s_Climber.lowerLeftClimber()));
    operator4.onFalse(new InstantCommand(() -> s_Climber.stopLeftClimber()));




    
  }



  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }



}
