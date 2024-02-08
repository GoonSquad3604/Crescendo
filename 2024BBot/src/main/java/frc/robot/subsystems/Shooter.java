// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  // Declares variables
  private CANSparkFlex leftShooterMotor;
  private CANSparkFlex rightShooterMotor;
  private CANSparkFlex indexMotor;
  private CANSparkFlex angleMotor;

  private RelativeEncoder leftShooterEncoder;
  private RelativeEncoder rightShooterEncoder;
  private RelativeEncoder indexEncoder;

  private SparkPIDController leftShooterPIDController;
  private SparkPIDController rightShooterPIDController;
  private SparkPIDController IndexPIDController;
  private SparkPIDController anglePIDController;

  private AbsoluteEncoder angleEncoder;

  private DigitalInput sensor;

  private static Shooter _instance;

  private int leftRPM = 5000;
  private int rightRPM = 5000;
  private int IndexRPM = 3000;

  /** Creates a new Shooter. */
  public Shooter() {

    leftShooterMotor = new CANSparkFlex(Constants.ShooterConstants.leftID, MotorType.kBrushless);
    rightShooterMotor = new CANSparkFlex(Constants.ShooterConstants.rightID, MotorType.kBrushless);
    angleMotor = new CANSparkFlex(Constants.ShooterConstants.angleID, MotorType.kBrushless);
    indexMotor = new CANSparkFlex(Constants.ShooterConstants.indexID, MotorType.kBrushless);

    sensor = new DigitalInput(0);

    leftShooterMotor.restoreFactoryDefaults();
    rightShooterMotor.restoreFactoryDefaults();
    angleMotor.restoreFactoryDefaults();
    indexMotor.restoreFactoryDefaults();

    anglePIDController = angleMotor.getPIDController();
    angleEncoder = angleMotor.getAbsoluteEncoder(Type.kDutyCycle);

    rightShooterEncoder = rightShooterMotor.getEncoder();
    rightShooterPIDController = rightShooterMotor.getPIDController();

    leftShooterEncoder = leftShooterMotor.getEncoder();
    leftShooterPIDController = leftShooterMotor.getPIDController();

    IndexPIDController = indexMotor.getPIDController();
    indexEncoder = indexMotor.getEncoder();

    // PIDS

    anglePIDController.setFeedbackDevice(angleEncoder);

    anglePIDController.setI(Constants.ShooterConstants.anglekI);
    anglePIDController.setD(Constants.ShooterConstants.anglekD);

    leftShooterPIDController.setP(Constants.ShooterConstants.shooterkP);
    leftShooterPIDController.setI(Constants.ShooterConstants.shooterkI);
    leftShooterPIDController.setD(Constants.ShooterConstants.shooterkD);
    leftShooterPIDController.setFF(Constants.ShooterConstants.shooterkF);

    rightShooterPIDController.setP(Constants.ShooterConstants.shooterkP);
    rightShooterPIDController.setI(Constants.ShooterConstants.shooterkI);
    rightShooterPIDController.setD(Constants.ShooterConstants.shooterkD);
    rightShooterPIDController.setFF(Constants.ShooterConstants.shooterkF);

    IndexPIDController.setP(Constants.ShooterConstants.indexkP);
    IndexPIDController.setI(Constants.ShooterConstants.indexkI);
    IndexPIDController.setD(Constants.ShooterConstants.indexkD);
    IndexPIDController.setFF(Constants.ShooterConstants.indexkF);

    rightShooterPIDController.setOutputRange(-1, 1);
    leftShooterPIDController.setOutputRange(-1, 1);
    anglePIDController.setOutputRange(-1, 1);
    IndexPIDController.setOutputRange(-1, 1);

    angleMotor.setIdleMode(IdleMode.kBrake);

    // Inverts left shooter motor
    leftShooterMotor.setInverted(true);
  }

  public static Shooter getInstance() {
    if (_instance == null) {
      _instance = new Shooter();
    }
    return _instance;
  }

  public void setPower(double speed) {
    // Sets motor speed
    leftShooterMotor.set(speed);
    rightShooterMotor.set(speed);
  }

  public void setShooterRPM() {
    leftShooterPIDController.setReference(leftRPM, ControlType.kVelocity);
    rightShooterPIDController.setReference(rightRPM, ControlType.kVelocity);
  }

  public void stopShooter() {
    leftShooterMotor.set(0);
    rightShooterMotor.set(0);
  }

  // ShooterAngle methods
  public void raiseAngle() {
    angleMotor.set(.2);
  }

  public void lowerAngle() {
    angleMotor.set(-.2);
  }

  public void stopAngle() {
    angleMotor.set(.0);
  }

  public double getShooterAngleClicks() {
    return angleEncoder.getPosition();
  }

  public void setUpP() {
    anglePIDController.setP(Constants.ShooterConstants.angleUpP);
  }

  public void setDownP() {
    anglePIDController.setP(Constants.ShooterConstants.angleDownP);
  }

  // Indexer methods
  public void setIndexRPM() {
    IndexPIDController.setReference(IndexRPM, ControlType.kVelocity);
  }

  public void setIndexPower(double speed) {
    indexMotor.set(speed);
  }

  public void indexStop() {
    indexMotor.set(0);
  }

  public boolean hasNote() {
    return !sensor.get();
  }

  public void shooterTo(double position) {
    anglePIDController.setReference(position / 360, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ShooterAngleEncoder", getShooterAngleClicks());
    SmartDashboard.putBoolean("sensor has target: ", hasNote());
  }
}
