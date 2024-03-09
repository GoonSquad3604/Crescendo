// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

  /*  Declare variables */

  private CANSparkFlex rightClimberMotor;
  private CANSparkFlex leftClimberMotor;

  private RelativeEncoder leftClimberEncoder;
  private RelativeEncoder rightClimberEncoder;

  private SparkPIDController rightClimberPIDController;
  private SparkPIDController leftClimberPIDController;

  private static Climber _instance;

  /** Creates a new Climber. */
  public Climber() {

    leftClimberMotor =
        new CANSparkFlex(Constants.ClimberConstants.leftClimbID, MotorType.kBrushless);

    rightClimberMotor =
        new CANSparkFlex(Constants.ClimberConstants.rightClimbID, MotorType.kBrushless);

    leftClimberMotor.restoreFactoryDefaults();
    rightClimberMotor.restoreFactoryDefaults();

    leftClimberEncoder = leftClimberMotor.getEncoder();
    leftClimberPIDController = leftClimberMotor.getPIDController();
    rightClimberEncoder = rightClimberMotor.getEncoder();
    rightClimberPIDController = rightClimberMotor.getPIDController();
    rightClimberPIDController.setFeedbackDevice(rightClimberEncoder);
    leftClimberPIDController.setFeedbackDevice(leftClimberEncoder);

    // PIDs
    leftClimberPIDController.setP(.5);
    rightClimberPIDController.setP(.5);

    leftClimberPIDController.setI(0);
    rightClimberPIDController.setI(0);

    leftClimberPIDController.setD(0);
    rightClimberPIDController.setD(0);

    leftClimberPIDController.setOutputRange(-.6, .6);
    rightClimberPIDController.setOutputRange(-.6, .6);

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

  public void zeroEncoders() {
    leftClimberEncoder.setPosition(0);
    rightClimberEncoder.setPosition(0);
  }

  public void climberUp() {
    leftClimberMotor.set(.4);
    rightClimberMotor.set(.4);
  }

  public void climberDown() {
    leftClimberMotor.set(-.4);
    rightClimberMotor.set(-.4);
  }

  public void raiseCimber() {
    leftClimberPIDController.setReference(
        Constants.ClimberConstants.maxLeftClimberHeight, ControlType.kPosition);
    rightClimberPIDController.setReference(
        Constants.ClimberConstants.maxRightClimberHeight, ControlType.kPosition);
  }

  public void lowerClimber() {
    leftClimberPIDController.setReference(
        Constants.ClimberConstants.leftClimbedPos, ControlType.kPosition);
    rightClimberPIDController.setReference(
        Constants.ClimberConstants.rightClimbedPos, ControlType.kPosition);
  }

  public void climbPos() {
    leftClimberPIDController.setReference(
        Constants.ClimberConstants.leftClimbedPos, ControlType.kPosition);
    rightClimberPIDController.setReference(
        Constants.ClimberConstants.rightClimbedPos, ControlType.kPosition);
  }

  public void stopClimber() {
    leftClimberMotor.set(0);
    rightClimberMotor.set(0);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("left climber height", leftClimberEncoder.getPosition());
    // SmartDashboard.putNumber("right climber height", rightClimberEncoder.getPosition());
    // SmartDashboard.putBoolean("STOP!!!!!!!!", (leftClimberEncoder.getPosition() <= 100));
  }
}
