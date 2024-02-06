// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.DutyCycleEncoder;



public class Climber extends SubsystemBase {


  private static Climber _instance;
  CANSparkMax rightMotor;
  CANSparkMax leftMotor;


  private SparkPIDController rightClimberPIDController;
  private SparkPIDController leftClimberPIDController;


  DutyCycleEncoder encoder;
  public double setSpeed, p, encoderSetPoint;
  



  /** Creates a new Climber. */
  public Climber() {
    leftMotor = new CANSparkMax(17, MotorType.kBrushless);
    rightMotor = new CANSparkMax(20, MotorType.kBrushless);

    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();

    
    
    leftClimberPIDController = leftMotor.getPIDController();
    rightClimberPIDController = rightMotor.getPIDController();

    leftClimberPIDController.setOutputRange(-.5, .5);
    rightClimberPIDController.setOutputRange(-.5, .5);

    leftClimberPIDController.setP(.5);
    rightClimberPIDController.setP(.5);

    leftClimberPIDController.setD(0);
    rightClimberPIDController.setD(0);

    leftClimberPIDController.setI(0);
    rightClimberPIDController.setI(0);
    

    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setIdleMode(IdleMode.kBrake);


  }



  public static Climber getInstance() {
    if (_instance == null) {
      _instance = new Climber();
  }
    return _instance;
  }



  /** Raises Left climber. */
  public void raiseLeftClimber(){
    leftMotor.set(-0.5);
  }


  /** Raises right climber. */
  public void raiseRightClimber(){
    rightMotor.set(-0.5);
  }


  /** Lowers left climber. */
  public void lowerLeftClimber(){
  leftMotor.set(0.5);
  }


  /** Lowers right climber. */
  public void lowerRightClimber(){
  rightMotor.set(0.5);
  }


  /** Stops left climber. */
  public void stopLeftClimber(){
    leftMotor.set(0);
  }

  /** Stops right climber */
  public void stopRightClimber(){
    rightMotor.set(0);
  }



  @Override
  public void periodic() {

  }




}