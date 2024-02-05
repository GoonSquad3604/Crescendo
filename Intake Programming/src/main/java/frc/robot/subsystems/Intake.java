// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
public class Intake extends SubsystemBase {

  
  
  private static Intake _instance;
  private static CANSparkMax intakeMotor;
  private static CANSparkMax leftHingeMotor;
  private static CANSparkMax rightHingeMotor;
  private SparkPIDController intakePIDController;
  private static SparkPIDController hingePIDController;
  /** Creates a new Intake. */
  public Intake() {

      leftHingeMotor = new CANSparkMax( 4, MotorType.kBrushless);
      rightHingeMotor = new CANSparkMax( 5, MotorType.kBrushless);
      intakeMotor = new CANSparkMax(6, MotorType.kBrushless);


    leftHingeMotor.follow(rightHingeMotor);

      //Intake PID values
    intakePIDController.setFF(Constants.IntakeConstants.kFF);
    intakePIDController.setP(Constants.IntakeConstants.kP);
    intakePIDController.setI(Constants.IntakeConstants.kI);
    intakePIDController.setD(Constants.IntakeConstants.kD);
    intakePIDController.setOutputRange(-1.0, 1.0);

      //Hinge PID values
    hingePIDController.setFF(Constants.HingeConstants.kFF);
    hingePIDController.setP(Constants.HingeConstants.kP);
    hingePIDController.setI(Constants.HingeConstants.kI);
    hingePIDController.setD(Constants.HingeConstants.kD);
    hingePIDController.setOutputRange(-1.0, 1.0);
  }
public static Intake getInstance() {
    
    if(_instance == null){
        _instance = new Intake();
    }
    return _instance;
}
    //stops the intake
   public static void stopIntake() {
    intakeMotor.set(0); 
   }

  //brings the game piece into robot
   public static void runIntake() {
    intakeMotor.set(Constants.IntakeConstants.intakeSpeed);
   }
  
   //vomits the game piece
   public static void vomit() {
    intakeMotor.set(Constants.IntakeConstants.vomitSpeed);
   }

   //raises the Hinge into the Intake
    public static void raiseHinge() {
       rightHingeMotor.set(Constants.HingeConstants.raiseHingeSpeed);
       leftHingeMotor.set(Constants.HingeConstants.raiseHingeSpeed);
    }
    //lowers the Hinge out of the Intake
    public static void lowerHinge() {
     rightHingeMotor.set(Constants.HingeConstants.lowerHingeSpeed);
     leftHingeMotor.set(Constants.HingeConstants.lowerHingeSpeed);
    }
    public static void stopHinge() {
     rightHingeMotor.set(0);
    leftHingeMotor.set(0);
    }
   public void setHinge(double leftPower, double rightPower) {
    rightHingeMotor.set(rightPower);
   }

   public static void hingePosition(double position) {
    hingePIDController.setReference(position, CANSparkMax.ControlType.kPosition);
   }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake P value", Constants.IntakeConstants.kP);
    SmartDashboard.putNumber("Intake FF value", Constants.IntakeConstants.kFF);
    SmartDashboard.putNumber("Intake Speed", Constants.IntakeConstants.intakeSpeed);
    SmartDashboard.putNumber("Hinge P value", Constants.HingeConstants.kP);
    SmartDashboard.putNumber("Hinge FF value", Constants.HingeConstants.kFF);
    SmartDashboard.putNumber("Raise Hinge Speed", Constants.HingeConstants.raiseHingeSpeed);
    SmartDashboard.putNumber("Lower Hinge Speed", Constants.HingeConstants.lowerHingeSpeed);
  }
}
