// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.CANAPITypes.CANDeviceType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Shooter extends SubsystemBase {

  private CANSparkFlex leftShooterMotor;
  private CANSparkFlex rightShooterMotor;
  private CANSparkFlex IndexShooterMotor;
  private CANSparkFlex AngleMotor;  
  private RelativeEncoder leftShooterEncoder;
  private RelativeEncoder rightShooterEncoder;
  private RelativeEncoder IndexShooterEncoder;
  private RelativeEncoder AngleEncoder;
  private SparkPIDController leftShooterPIDController;
  private SparkPIDController rightShooterPIDController;
  private SparkPIDController IndexShooterPIDController;
  private SparkPIDController AnglePIDController;


  private static Shooter _instance;


  private double kP,kI,kD,kFF,AnglekP,AngleI,AnglekD;
  private double AnglePos;
  private int leftRPM;
  private int rightRPM;
  private int IndexRPM;
  /** Creates a new Shooter. */
  public Shooter() {
  
      leftShooterMotor = new CANSparkFlex(0, MotorType.kBrushless);
      leftShooterEncoder = leftShooterMotor.getEncoder();
      leftShooterPIDController = leftShooterMotor.getPIDController();

      rightShooterMotor = new CANSparkFlex(1, MotorType.kBrushless);
      rightShooterEncoder = rightShooterMotor.getEncoder();
      rightShooterPIDController = rightShooterMotor.getPIDController();
      
      leftShooterMotor.restoreFactoryDefaults();
      rightShooterMotor.restoreFactoryDefaults();
      //PIDS
      AnglePIDController.setP(Constants.ShooterConstants.AnglekP);
      AnglePIDController.setI(Constants.ShooterConstants.AnglekI);
      AnglePIDController.setD(Constants.ShooterConstants.AnglekD);
      leftShooterPIDController.setP(Constants.ShooterConstants.kP);
      leftShooterPIDController.setI(Constants.ShooterConstants.kI);
      leftShooterPIDController.setD(Constants.ShooterConstants.kD);
      leftShooterPIDController.setFF(Constants.ShooterConstants.kFF);
      rightShooterPIDController.setP(Constants.ShooterConstants.kP);
      rightShooterPIDController.setI(Constants.ShooterConstants.kI);
      rightShooterPIDController.setD(Constants.ShooterConstants.kD);
      rightShooterPIDController.setFF(Constants.ShooterConstants.kFF);

      //Inverts left shooter motor
      leftShooterMotor.setInverted(true);
  }

 public static Shooter getInstance() {
      if(_instance == null){
        _instance = new Shooter();
      }
      return _instance;
 }
//sets shooter power
 public void setPower() {

 }
public void runShooter(double speed) {
  //Sets motor speed
  leftShooterMotor.set(speed);
  rightShooterMotor.set(speed);
}
public void setRPM(){
  leftShooterPIDController.setReference(leftRPM, ControlType.kVelocity);
  rightShooterPIDController.setReference(rightRPM, ControlType.kVelocity);
  AnglePIDController.setReference(AnglePos, ControlType.kPosition);
}

  @Override
  public void periodic() {
    //Smart dashboard stuff
    double leftRPM = SmartDashboard.getNumber("leftRPM", 1000);
    SmartDashboard.putNumber("leftRPM", leftRPM);

    double rightRPM = SmartDashboard.getNumber("rightRPM", 1000);
    SmartDashboard.putNumber("rightRPM", rightRPM);
    
    SmartDashboard.getNumber("kD", 0);

    double newShootP = SmartDashboard.getNumber("kP", 0);
    if (newShootP!=kP){
      kP=newShootP;
    }
    SmartDashboard.putNumber("kP",kP);
    

    double newShootFF = SmartDashboard.getNumber("kFF", 0);    
    if(newShootFF != kFF){
      kFF = newShootFF;
    }
    SmartDashboard.putNumber("kD", kD);
    //Seperate PID for Angle Position
    double newAngleP = SmartDashboard.getNumber("AnglekP", 0);
    if (newAngleP!=AnglekP){
      AnglekP=newAngleP;
    }
    SmartDashboard.putNumber("AnglekP",AnglekP);
    double newAnglekD = SmartDashboard.getNumber("AnglekD", 0);
    if(newAnglekD!=AnglekD){
      AnglekD=newAnglekD;
    }
    SmartDashboard.putNumber("AnglekD", AnglekD);
    // This method will be called once per scheduler run
  }
}


//hello if you are reading this all the thing below this is lucas's problem
//never gonna give you up, never gonna let you down -Rick Astley
//cheese :)
// "I am a bozo" -not tom / Andrew
//"True " -lucas