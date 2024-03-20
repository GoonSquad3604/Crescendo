// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Flipper extends SubsystemBase {
  private static Flipper _instance;
  private CANSparkMax flipperMotor;
  private double flipperD, flipperP, flipperI;
  private SparkPIDController flipperPIDController;
  private AbsoluteEncoder angleEncoder;
  private double setFlipper;

  /** Creates a new Flipper. */
  public Flipper() {
    flipperMotor = new CANSparkMax(Constants.FlipperConstants.flipperID, MotorType.kBrushless);
    flipperMotor.restoreFactoryDefaults();

    angleEncoder = flipperMotor.getAbsoluteEncoder(Type.kDutyCycle);
    flipperPIDController = flipperMotor.getPIDController();
    flipperPIDController.setFeedbackDevice(angleEncoder);

    flipperPIDController.setP(2.2); // geometry dash
    flipperPIDController.setI(0);
    flipperPIDController.setD(0);

    flipperPIDController.setOutputRange(-.4, .25);

    flipperMotor.enableVoltageCompensation(12);
  }

  public void runFlipper() {
    flipperMotor.set(0.1);
  }

  public void setFlipperUp() {
    flipperPIDController.setReference(.35, ControlType.kPosition);
  }

  public void setFlipperDown() {
    flipperPIDController.setReference(.75, ControlType.kPosition);
  }

  public void runFlipperBackward() {
    flipperMotor.set(-0.1);
  }

  public void stopFlipper() {
    flipperMotor.set(0);
  }

  public void panic() {
    flipperPIDController.setReference(.23, ControlType.kPosition);
  } // aaaaaaaaaaaaaaaaa

  public static Flipper getInstance() {

    if (_instance == null) {
      _instance = new Flipper();
    }
    return _instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("appliedOutput", flipperMotor.getAppliedOutput());
    // double newFlipperkD = SmartDashboard.getNumber("FlipperkD", 0);
    // if (newFlipperkD != flipperD) {
    //   flipperD = newFlipperkD;
    //   flipperPIDController.setD(flipperD);
    // }
    // SmartDashboard.putNumber("FlipperkD", flipperD);

    // double newFlipperI = SmartDashboard.getNumber("FlipperkI", 0);
    // if (newFlipperI != flipperI) {
    //   flipperI = newFlipperI;
    //   flipperPIDController.setI(flipperI);
    // }
    // SmartDashboard.putNumber("FlipperkI", flipperI);

    //  double newFlipperP = SmartDashboard.getNumber("FlipperkP", 1);
    //   if(newFlipperP!=flipperP){
    //     flipperP=newFlipperP;
    //     flipperPIDController.setP(flipperP);
    //   }
    //   SmartDashboard.putNumber("FlipperkP", flipperP);

    // SmartDashboard.putNumber("FlipperPosition", angleEncoder.getPosition());

    // double newSetFlipper = SmartDashboard.getNumber("setFlipper", 0);
    // if (newSetFlipper != setFlipper) {
    //   setFlipper = newSetFlipper;
    // }
    // SmartDashboard.putNumber("setFlipper", setFlipper);
  }
}
