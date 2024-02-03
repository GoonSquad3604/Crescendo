
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drive.Aim;
import frc.robot.commands.drive.AimPID;
import frc.robot.commands.drive.OneMeterFromTag;
import frc.robot.commands.drive.SwerveDefaultDrive;
import frc.robot.commands.shooter.FeedUntillSensor;
import frc.robot.commands.shooter.RepositionNote;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

public class RobotContainer {

  /* Controllers */
  private XboxController driver = new XboxController(0);  

  /* Driver Buttons */
    
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton driverX = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton driverB = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton driverY = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton driverA = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton driverRightBumper = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

    private final SendableChooser<Command> autoChooser;

  /* Subsystems */
  private final SwerveDrive s_Swerve = SwerveDrive.getInstance();
  //private final Shooter s_Shooter = Shooter.getInstance();
  //private Vision s_Vision = Vision.getInstance();
  public RobotContainer() {

    //NamedCommands.registerCommand("RunIntake", new InstantCommand(() -> s_Shooter.spin()));
    //NamedCommands.registerCommand("Shoot", InstantCommand(() -> s_Shooter.spin()));


    /* Default Commands */
    s_Swerve.setDefaultCommand(
      new SwerveDefaultDrive(
        () -> -driver.getLeftY(), 
        () -> -driver.getLeftX(), 
        () -> driver.getRightX(), 
        () -> driver.getLeftTriggerAxis(),
        robotCentric)
    );

    configureBindings();
    
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);

  }

  private void configureBindings() {

    SmartDashboard.putData("MoveForward", new PathPlannerAuto("MoveForward"));

    JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);

    /* Driver Buttons */
     zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
    // driverX.onTrue(new Aim(s_Swerve));
    // driverB.onTrue(new Aim2PID(s_Swerve, driver, s_Vision));
    // driverA.onTrue(new SequentialCommandGroup(new FeedUntillSensor(), new RepositionNote()));
  
    // driverX.onTrue(new InstantCommand(() -> s_Shooter.indexStop()));
  
    // driverB.toggleOnTrue(new InstantCommand(() -> s_Shooter.spin()));
        
    // driverY.toggleOnFalse(new InstantCommand(() -> s_Shooter.stop()));

    // driverRightBumper.onTrue(new InstantCommand(() -> s_Shooter.indexPower()));

    //driverA.onTrue(new InstantCommand(() -> s_Shooter.spin(1)));
    //driverA.onFalse(new InstantCommand(() -> s_Shooter.stop()));


  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
}
}