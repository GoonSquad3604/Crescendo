// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.commands.autons.TestAuton;
// import frc.robot.commands.drive.Aim;
// import frc.robot.commands.drive.AutoBalance;
// import frc.robot.commands.drive.CenterPole;
// import frc.robot.commands.drive.DefaultAngle;
// import frc.robot.commands.drive.SwerveDefaultDrive;
// import frc.robot.commands.intake.ToggleHinge;
// import frc.robot.commands.states.SetConeMode;
// import frc.robot.commands.states.SetCubeMode;
// import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  //Declare controllers
  private XboxController driver = new XboxController(0);
  private XboxController operatorController = new XboxController(1);
  private Joystick operatorJoystick = new Joystick(2);
  //TestAuton auton = new TestAuton();



  
  //Declare Certain Buttons
  private JoystickButton driverLeftBumber = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private JoystickButton driverRightBumper = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  private JoystickButton operatorLeftBumper = new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value);
  

  //Declare Subsystems
  private SwerveDrive s_SwerveDrive = SwerveDrive.getInstance();
  //private Intake s_Intake = Intake.getInstance();
 // private StateController s_StateController = StateController.getInstance();
  
  private Vision s_Vision = new Vision();

  //private Arm s_Arm = Arm.getInstance();
  //rivate Shoulder s_Shoulder = Shoulder.getInstance();
  //private LED s_LED = new LED(Constants.LEDConstants.led1, 26);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //double speedBoost = 1.0;


    //Set Defualt Commands%
    s_SwerveDrive.setDefaultCommand(
            new SwerveDefaultDrive(() -> driver.getLeftY(), () -> driver.getLeftX(), () -> driver.getRightX(), driverLeftBumber, driverRightBumper, () -> driver.getLeftTriggerAxis()));


    //s_Arm.setDefaultCommand(new ArmDefaultCommand(() -> operatorController.getLeftY(), () -> operatorController.getRightY(), operatorLeftBumper));

    configureBindings();

    //SmartDashboard.putString("auton pose", auton.getInitialPose().toString());
    //s_SwerveDrive.resetOdometry(auton.getInitialPose());
    // s_SwerveDrive.resetModulesToAbsolute();
  }


  
  private void configureBindings() {

    // Driver
    JoystickButton driverY = new JoystickButton(driver, XboxController.Button.kY.value);
    JoystickButton driverB = new JoystickButton(driver, XboxController.Button.kB.value);
    JoystickButton driverA = new JoystickButton(driver, XboxController.Button.kA.value);
    JoystickButton driverX = new JoystickButton(driver, XboxController.Button.kX.value);

    // Operator
    JoystickButton operatorY = new JoystickButton(operatorController, XboxController.Button.kY.value);
    JoystickButton operatorX = new JoystickButton(operatorController, XboxController.Button.kX.value);
    JoystickButton operatorA = new JoystickButton(operatorController, XboxController.Button.kA.value);
    JoystickButton operatorLeftBumper = new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value);
    JoystickButton operatorRightBumper = new JoystickButton(operatorController, XboxController.Button.kRightBumper.value);
    
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



  
    driverY.onTrue(new InstantCommand(() -> s_SwerveDrive.zeroGyro()));
    driverB.onTrue(new DefaultAngle(s_SwerveDrive, driver));
    // driverX.onTrue(new AutoBalance(s_SwerveDrive));
    
    driverA.onTrue(new Aim());
    //driverLeftBumper.whileTrue(() -> (speedBoost = 0.5;)); 


    // operatorY.onTrue(new InstantCommand(() -> s_Intake.setHinge(0.2, 0.2)));
    // operatorY.onFalse(new InstantCommand(() -> s_Intake.setHinge(0, 0)));
    // operatorA.onTrue(new InstantCommand(() -> s_Intake.setHinge(-0.15, -0.15)));
    // operatorA.onFalse(new InstantCommand(() -> s_Intake.setHinge(0, 0)));

    
    // operator1.onTrue((new InstantCommand(() -> s_Intake.runIntake())));
    // operator1.onFalse((new InstantCommand(() -> s_Intake.stopIntake())));
    // operator6.onTrue((new InstantCommand(() -> s_Intake.vomit())));
    // operator6.onFalse((new InstantCommand(()-> s_Intake.stopIntake())));

    // operator2.onTrue(new InstantCommand(() -> s_Intake.setHinge(0.2, 0.2)));
    // operator2.onFalse(new InstantCommand(() -> s_Intake.setHinge(0, 0)));
    // operator2.onTrue(new ToggleHinge());
    // operator3.onTrue(new InstantCommand(() -> s_Intake.setHinge(-0.2, -0.2)));
    // operator3.onFalse(new InstantCommand(() -> s_Intake.setHinge(0, 0)));



    // operator7.onTrue(new SetCubeMode(s_LED));
    // operator8.onTrue(new SetConeMode(s_LED));

    // // Claw PID
    // //operator9.onTrue(new InstantCommand(() -> s_Arm.clawTo(0)));
    // operator4.onTrue(new InstantCommand(() -> s_Arm.clawTo(s_StateController.getClosedClawPos())));
    // operator5.onTrue(new InstantCommand(() -> s_Arm.clawTo(0)));
    // operator4.onTrue(new InstantCommand(() -> s_Arm.moveClaw(-0.2)));
    // operator5.onTrue(new InstantCommand(() -> s_Arm.moveClaw(0.2)));

    // Arm Positions

    // operator9.onTrue(new ReadyToRecieve());

    // // Arm high
    // operator10.onTrue(new ArmHigh(s_StateController.getHighPosShoulder(), s_StateController.getHighPosShoulder()));

    // // Arm mid
    // operator11.onTrue(new ArmMedium(s_StateController.getMidPosShoulder(), s_StateController.getMidPosShoulder()));

    // // Arm low
    // operator12.onTrue(new ArmLow());


    // // operator10.onTrue(new ArmHigh(s_StateController));
    // operator8.onTrue(new SetConeMode(s_LED));
    // operatorLeftBumper.onFalse(new InstantCommand(() -> s_Arm.setElbow(0)));
    // operatorLeftBumper.onFalse(new InstantCommand(() -> s_Shoulder.setShoulder(0)));
    


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   TestAuton auton = new TestAuton();
  //   s_SwerveDrive.resetOdometry(auton.getInitialPose());
  //   return auton;
  // }

}













//ur mom