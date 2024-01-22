// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SwerveDefaultDrive;
import frc.robot.subsystems.SwerveDrive;

public class RobotContainer {

  /* Controllers */
  private XboxController driver = new XboxController(0);  

  /* Driver Buttons */
    
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

  /* Subsystems */
  private final SwerveDrive s_Swerve = SwerveDrive.getInstance();

  public RobotContainer() {

    /* Default Commands */
    s_Swerve.setDefaultCommand(
      new SwerveDefaultDrive(
        () -> driver.getLeftY(), 
        () -> driver.getLeftX(), 
        () -> driver.getRightX(), 
        robotCentric)
    );

    configureBindings();
  }

  private void configureBindings() {

    JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);

    /* Driver Buttons */
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

    
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
