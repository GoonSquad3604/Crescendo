// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class Shooter extends SubsystemBase {
 
  private static Shooter _instance;
  private static final int leftID =1;
  private static final int rightID = 2;

  private CANSparkFlex leftMotor;
  private CANSparkFlex rightMotor;

  private SparkPIDController leftController;
  private SparkPIDController rightController;

  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;
  
  private double leftPower, kP, rightPower, kD, kI, kF;


  /** Creates a new Shooter. */
  public Shooter() {
    kP=.0005;
    leftPower =1000;
    rightPower =1000;
    kD=0;
    kI=0;
    kF = 0.00016;

    leftMotor = new CANSparkFlex(leftID, MotorType.kBrushless);
    rightMotor = new CANSparkFlex(rightID, MotorType.kBrushless);

    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();

    rightMotor.setInverted(true);

    leftController = leftMotor.getPIDController();
    rightController = rightMotor.getPIDController();

    leftEncoder = leftMotor.getEncoder();
    rightEncoder = rightMotor.getEncoder();

    leftController.setP(kP);

    leftController.setI(kI);
    leftController.setD(kD);
    leftController.setOutputRange(-1, 1);
    leftController.setFF(kF);

    rightController.setP(kP);

    rightController.setI(kI);
    rightController.setD(kD);
    rightController.setOutputRange(-1, 1);
    rightController.setFF(kF);

    leftMotor.setIdleMode(IdleMode.kCoast);
    rightMotor.setIdleMode(IdleMode.kCoast);


  }

  public static Shooter getInstance(){
    if(_instance == null){
      _instance = new Shooter();
    }
    return _instance;
  }

  public void spin(){
    leftController.setReference(leftPower,CANSparkFlex.ControlType.kVelocity);
    rightController.setReference(rightPower, CANSparkFlex.ControlType.kVelocity);
  }
  public void stop(){
    leftMotor.set(0);
    rightMotor.set(0);
  }
  public void spin(double speed){
    leftMotor.set(speed);
    rightMotor.set(speed);
  }
  @Override
  public void periodic() {
    double leftSpeed = leftEncoder.getVelocity();
    double rightSpeed = rightEncoder.getVelocity();
    

    leftPower = SmartDashboard.getNumber("input left speed", 1000);
      SmartDashboard.putNumber("input left speed", leftPower);

    rightPower = SmartDashboard.getNumber("input right speed", 1000);
          SmartDashboard.putNumber("input right speed", rightPower);

    SmartDashboard.putNumber("leftSpeed", leftSpeed);
    SmartDashboard.putNumber("rightSpeed", rightSpeed);

    double newP = SmartDashboard.getNumber("Put in kp ", .0005);

    if(newP!=kP){
      kP=newP;
          leftController.setP(kP);
          rightController.setP(kP);


    }
        SmartDashboard.putNumber("Put in kp ", kP);
    double newD = SmartDashboard.getNumber("Put in kD ", .0);

    if(newD!=kD){
      kD=newD;
          leftController.setD(kD);
          rightController.setD(kD);


    }
        SmartDashboard.putNumber("Put in kD ", kD);

double newI = SmartDashboard.getNumber("Put in kI ", 0);

    if(newI!=kI){
      kI=newI;
          leftController.setP(kI);
          rightController.setP(kI);


    }
        SmartDashboard.putNumber("Put in kI ", kI);

  double newF = SmartDashboard.getNumber("Put in kF ", 0.00016);

    if(newF!=kF){
      kF=newF;
          leftController.setFF(kF);
          rightController.setFF(kF);


    }
        SmartDashboard.putNumber("Put in kF ", kF);
    // This method will be called once per scheduler run
  }
}
