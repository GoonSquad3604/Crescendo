// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class rAMP extends SubsystemBase {
  private static rAMP _instance;
  private AbsoluteEncoder rAMPEncoder;
  private CANSparkMax rAMPMotor;
  private double rAMPP, rAMPI, rAMPD;
  private SparkPIDController rAMPPIDController;

  public rAMP() {
    rAMPMotor = new CANSparkMax(Constants.rAMPConstants.rAMPID, MotorType.kBrushless);
    rAMPEncoder = rAMPMotor.getAbsoluteEncoder(Type.kDutyCycle);
    rAMPPIDController = rAMPMotor.getPIDController();
    rAMPPIDController.setFeedbackDevice(rAMPEncoder);

    rAMPPIDController.setP(2.3);
    rAMPPIDController.setI(0);
    rAMPPIDController.setD(0);

    rAMPPIDController.setOutputRange(-.1, .1);

  }

  public static rAMP getInstance(){
    if(_instance == null){
      _instance = new rAMP();
    }
    return _instance;

  }
  public void runrAMP() {
    rAMPMotor.set(.2);
  }
  public void runrAMPBack() {
    rAMPMotor.set(-.2);
  }
  public void stoprAMP() {
    rAMPMotor.set(.0);
  }
  public void setrAMPUp() {
    rAMPPIDController.setReference(Constants.rAMPConstants.rAMPUP, ControlType.kPosition);
  }
public void setrAMPDown() {
    rAMPPIDController.setReference(Constants.rAMPConstants.rAMPDOWN, ControlType.kPosition);
  }
  public void setrAMPTO(double pos) {
    rAMPPIDController.setReference(pos, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("rampPos", rAMPEncoder.getPosition());
  }
}
