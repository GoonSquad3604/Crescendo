// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private static Intake _instance;
  private CANSparkFlex intakeMotor;
  private CANSparkFlex hingeMotor;
  private AbsoluteEncoder hingeEncoder;
  private SparkPIDController intakePIDController;
  private SparkPIDController hingePIDController;

  private double p, i, d;

  /** Creates a new Intake. */
  public Intake() {
    p = 5;
    i = 0;
    d = 0;
    hingeMotor = new CANSparkFlex(Constants.IntakeConstants.hingeID, MotorType.kBrushless);
    intakeMotor = new CANSparkFlex(Constants.IntakeConstants.intakeID, MotorType.kBrushless);

    intakePIDController = intakeMotor.getPIDController();
    // leftHingeMotor.follow(rightHingeMotor);
    // rightHingeMotor.setInverted(true);

    intakeMotor.restoreFactoryDefaults();
    hingeMotor.restoreFactoryDefaults();
    hingePIDController = hingeMotor.getPIDController();
    hingeEncoder = hingeMotor.getAbsoluteEncoder(Type.kDutyCycle);

    hingePIDController.setFeedbackDevice(hingeEncoder);

    // Intake PID values
    intakePIDController.setFF(Constants.IntakeConstants.intakekFF);
    intakePIDController.setP(Constants.IntakeConstants.intakekP);
    intakePIDController.setI(Constants.IntakeConstants.intakekI);
    intakePIDController.setD(Constants.IntakeConstants.intakekD);
    intakePIDController.setOutputRange(-1.0, 1.0);

    // Hinge PID values
    hingePIDController.setP(5);
    hingePIDController.setI(i);
    hingePIDController.setD(d);
    hingePIDController.setOutputRange(-.6, .6);
  }

  public static Intake getInstance() {

    if (_instance == null) {
      _instance = new Intake();
    }
    return _instance;
  }

  // stops the intake
  public void stopIntake() {
    intakeMotor.set(0);
  }

  public void intakeRPM() {
    intakePIDController.setReference(Constants.IntakeConstants.intakeRPM, ControlType.kVelocity);
  }

  // brings the game piece into robot
  public void runIntake() {
    intakeMotor.set(1);
  }

  // vomits the game piece
  public void vomit() {
    intakeMotor.set(-1);
  }

  // raises the Hinge into the Intake
  public void raiseHinge() {
    hingeMotor.set(.4);
  }

  // lowers the Hinge out of the Intake
  public void lowerHinge() {
    hingeMotor.set(-.4);
  }

  public void stopHinge() {
    hingeMotor.set(0);
  }

  public void setHingeTo(double hingePos) {
    hingePIDController.setReference(hingePos, ControlType.kPosition);

    System.out.println(hingePos);
  }

  // public void hingePosition(double position) {
  //   hingePIDController.setReference(position, CANSparkMax.ControlType.kPosition);
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Intake P value", Constants.IntakeConstants.kP);
    // SmartDashboard.putNumber("Intake FF value", Constants.IntakeConstants.kFF);
    // SmartDashboard.putNumber("Intake Speed", Constants.IntakeConstants.intakeSpeed);
    // SmartDashboard.putNumber("Hinge P value", Constants.HingeConstants.kP);
    // SmartDashboard.putNumber("Hinge FF value", Constants.HingeConstants.kFF);
    // SmartDashboard.putNumber("Raise Hinge Speed", Constants.HingeConstants.raiseHingeSpeed);
    // SmartDashboard.putNumber("Lower Hinge Speed", Constants.HingeConstants.lowerHingeSpeed);

    SmartDashboard.putNumber("hinge position", hingeEncoder.getPosition());
  }
}
