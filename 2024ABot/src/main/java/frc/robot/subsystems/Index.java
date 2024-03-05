// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Index extends SubsystemBase {
  /** Creates a new Index. */
  private static Index _instance;

  private CANSparkFlex indexMotor;
  private RelativeEncoder indexEncoder;
  private SparkPIDController indexPIDController;
  private DigitalInput sensor;

  private double indexRPM;
  public Index() {
    indexMotor = new CANSparkFlex(Constants.IndexConstants.indexID, MotorType.kBrushless);
    indexMotor.restoreFactoryDefaults();

    sensor = new DigitalInput(0);

    indexPIDController = indexMotor.getPIDController();
    indexEncoder = indexMotor.getEncoder();
    indexPIDController.setFeedbackDevice(indexEncoder);
    indexPIDController.setP(Constants.IndexConstants.indexkP);
    indexPIDController.setI(Constants.IndexConstants.indexkI);
    indexPIDController.setD(Constants.IndexConstants.indexkD);
    indexPIDController.setFF(Constants.IndexConstants.indexkF);


    indexPIDController.setOutputRange(-1, 1);
    indexMotor.setInverted(true);

  }

  public static Index getInstance() {
    if (_instance == null) {
      _instance = new Index();
    }
    return _instance;
  }
  public void setIndexRPM(double RPM) {
    indexPIDController.setReference(RPM, ControlType.kVelocity);
  }
  public void setIndexPower(double speed) {
    indexMotor.set(-speed);
  }

  public void indexStop() {
    indexMotor.set(0);
  }
  public boolean hasNote() {
    return !sensor.get();
  }

    


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("hasNote", hasNote());
  }
}
