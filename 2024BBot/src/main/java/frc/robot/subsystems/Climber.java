// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;



public class Climber extends SubsystemBase {

/*  Declare variables */

  private CANSparkMax rightClimberMotor;
  private CANSparkMax leftClimberMotor;

  private RelativeEncoder leftClimberEncoder;
  private RelativeEncoder rightClimberEncoder;

  private SparkPIDController rightClimberPIDController;
  private SparkPIDController leftClimberPIDController;

  private static Climber _instance;
  
  private int leftMaxHeight;
  private int rightMaxHeight;

  /** Creates a new Climber. */
  public Climber() {

    leftClimberMotor = new CANSparkMax(Constants.ClimberConstants.leftClimbID, MotorType.kBrushless);
    leftClimberEncoder = leftClimberMotor.getEncoder();
    leftClimberPIDController = leftClimberMotor.getPIDController();

    rightClimberMotor = new CANSparkMax(Constants.ClimberConstants.rightClimbID, MotorType.kBrushless);
    rightClimberEncoder = rightClimberMotor.getEncoder();
    rightClimberPIDController = rightClimberMotor.getPIDController();

    leftClimberMotor.restoreFactoryDefaults();
    rightClimberMotor.restoreFactoryDefaults();

    
    //PIDs
    leftClimberPIDController.setP(.5);
    rightClimberPIDController.setP(.5);

    leftClimberPIDController.setI(0);
    rightClimberPIDController.setI(0);

    leftClimberPIDController.setD(0);
    rightClimberPIDController.setD(0);


    leftClimberPIDController.setOutputRange(-.5, .5);
    rightClimberPIDController.setOutputRange(-.5, .5);


    leftClimberMotor.setIdleMode(IdleMode.kBrake);
    rightClimberMotor.setIdleMode(IdleMode.kBrake);

    zeroEncoders();

  }


  public static Climber getInstance() {
    if (_instance == null) {
      _instance = new Climber();
  }
    return _instance;
  }

  public void zeroEncoders(){
    leftClimberEncoder.setPosition(0);
    rightClimberEncoder.setPosition(0);
  }

  public void climberUp(){
    leftClimberMotor.set(.3);
    rightClimberMotor.set(.3);
  }
  public void climberDown(){
    leftClimberMotor.set(-.3);
    rightClimberMotor.set(-.3);
  }


 public void raiseCimber(){
  leftClimberPIDController.setReference(leftMaxHeight, ControlType.kPosition);
  rightClimberPIDController.setReference(rightMaxHeight, ControlType.kPosition);
 }

 public void lowerClimber(){
  leftClimberPIDController.setReference(leftMaxHeight, ControlType.kPosition);
  rightClimberPIDController.setReference(rightMaxHeight, ControlType.kPosition);
 }


 public void stopClimber(){
  leftClimberMotor.set(0);
  rightClimberMotor.set(0);
}




  @Override
  public void periodic() {
   SmartDashboard.putNumber("left climber height", leftClimberEncoder.getPosition());
   SmartDashboard.putNumber("right climber height", rightClimberEncoder.getPosition());
  }

}