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
  private CANSparkFlex angleMotor;

  private RelativeEncoder leftShooterEncoder;
  private RelativeEncoder rightShooterEncoder;
  private RelativeEncoder indexEncoder;

  private SparkPIDController leftShooterPIDController;
  private SparkPIDController rightShooterPIDController;
  private SparkPIDController anglePIDController;

  private AbsoluteEncoder angleEncoder;


  private static Shooter _instance;
  private double angleD, angleP, angleI, kPLeft, kFFLeft, kPRight, kFFRight;

  private double leftRPM = 4000;
  private double rightRPM = 6500;
  private double trapAngle = 48;

  /** Creates a new Shooter. */
  public Shooter() {

    leftShooterMotor = new CANSparkFlex(Constants.ShooterConstants.leftID, MotorType.kBrushless);
    rightShooterMotor = new CANSparkFlex(Constants.ShooterConstants.rightID, MotorType.kBrushless);
    angleMotor = new CANSparkFlex(Constants.ShooterConstants.angleID, MotorType.kBrushless);


    leftShooterMotor.restoreFactoryDefaults();
    rightShooterMotor.restoreFactoryDefaults();
    angleMotor.restoreFactoryDefaults();

    anglePIDController = angleMotor.getPIDController();
    angleEncoder = angleMotor.getAbsoluteEncoder(Type.kDutyCycle);

    rightShooterEncoder = rightShooterMotor.getEncoder();
    rightShooterPIDController = rightShooterMotor.getPIDController();

    leftShooterEncoder = leftShooterMotor.getEncoder();
    leftShooterPIDController = leftShooterMotor.getPIDController();

    

    // PIDS

    anglePIDController.setFeedbackDevice(angleEncoder);

    // anglePIDController.setI(Constants.ShooterConstants.anglekI);
    // anglePIDController.setD(Constants.ShooterConstants.anglekD);

    anglePIDController.setP(7);
    anglePIDController.setI(angleI);
    anglePIDController.setD(angleD);

    leftShooterPIDController.setP(Constants.ShooterConstants.shooterkP);
    leftShooterPIDController.setI(Constants.ShooterConstants.shooterkI);
    leftShooterPIDController.setD(Constants.ShooterConstants.shooterkD);
    leftShooterPIDController.setFF(Constants.ShooterConstants.shooterkF);

    rightShooterPIDController.setP(Constants.ShooterConstants.shooterkP);
    rightShooterPIDController.setI(Constants.ShooterConstants.shooterkI);
    rightShooterPIDController.setD(Constants.ShooterConstants.shooterkD);
    rightShooterPIDController.setFF(Constants.ShooterConstants.shooterkF);

    

    rightShooterPIDController.setOutputRange(-1, 1);
    leftShooterPIDController.setOutputRange(-1, 1);
    anglePIDController.setOutputRange(-1, 1);

    leftShooterMotor.enableVoltageCompensation(12);
    rightShooterMotor.enableVoltageCompensation(12);

    angleMotor.setIdleMode(IdleMode.kBrake);
      }

  public static Shooter getInstance() {
    if (_instance == null) {
      _instance = new Shooter();
    }
    return _instance;
  }

  public void setPower(double speed) {
    // Sets motor speed
    leftShooterMotor.set(-speed);
    rightShooterMotor.set(speed);
  }

  public void setShooterRPMSpeaker() {

    // if (leftShooterPIDController.getP() != kPLeft) {
    //   leftShooterPIDController.setP(kPLeft);
    //   rightShooterPIDController.setP(kPRight);
    // }

    // if (leftShooterPIDController.getFF() != kFF) {
    //   leftShooterPIDController.setFF(kFF);
    //   rightShooterPIDController.setFF(kFF);
    // }

    leftShooterPIDController.setReference(-leftRPM, ControlType.kVelocity);
    rightShooterPIDController.setReference(rightRPM, ControlType.kVelocity);
  }

  public void setShooterRPMAMP() {
    double kP = .0005;
    double kFF = .00016;
    if (leftShooterPIDController.getP() != kP) {
      leftShooterPIDController.setP(kP);
      rightShooterPIDController.setP(kP);
    }
    if (leftShooterPIDController.getFF() != kFF) {
      leftShooterPIDController.setFF(kFF);
      rightShooterPIDController.setFF(kFF);
    }
    leftShooterPIDController.setReference(0, ControlType.kVelocity);
    rightShooterPIDController.setReference(1000, ControlType.kVelocity);
  }

  public void setShooterRPM(double leftRPM, double rightRPM) {
    leftShooterPIDController.setReference(leftRPM, ControlType.kVelocity);
    rightShooterPIDController.setReference(rightRPM, ControlType.kVelocity);
  }

  public void babyBird(double leftRPM, double rightRPM) {
    leftShooterPIDController.setReference(leftRPM, ControlType.kVelocity);
    rightShooterPIDController.setReference(-rightRPM, ControlType.kVelocity);
  }

  public void stopShooter() {
    leftShooterMotor.set(0);
    rightShooterMotor.set(0);
  }
  public void stopShooterRPM() {
   leftShooterPIDController.setReference(0, ControlType.kVelocity);
    rightShooterPIDController.setReference(-0, ControlType.kVelocity);
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

  

  

  
  // -0.002667*X + 0.7915
  public void shooterTo(double position) {
    anglePIDController.setReference(position * .00297 + .49717, ControlType.kPosition);
  }

  public void shooterTo() {
    anglePIDController.setReference(trapAngle * .00297 + .49717, ControlType.kPosition);
  }

  public void shooterToPos(double pos) {
    anglePIDController.setReference(pos, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("ShooterAngleEncoder", getShooterAngleClicks());
   // SmartDashboard.putNumber("ShooterAngle", (getShooterAngleClicks() - .49717) / .00297);

    // SmartDashboard.putNumber("leftShooterRPM", leftShooterEncoder.getVelocity());
    // SmartDashboard.putNumber("rightShooterRPM", rightShooterEncoder.getVelocity());

    // SmartDashboard.putBoolean("isInverted", angleMotor.getInverted());
    // double newAngleP = SmartDashboard.getNumber("AnglekP", 7);
    // if (newAngleP != angleP) {
    //   angleP = newAngleP;
    //   anglePIDController.setP(angleP);
    // }
    // leftRPM = SmartDashboard.getNumber("leftRPM", 4000);
    // SmartDashboard.putNumber("leftRPM", leftRPM);

    // rightRPM = SmartDashboard.getNumber("rightRPM", 6500);
    // SmartDashboard.putNumber("rightRPM", rightRPM);

    // double newLeftSpinP = SmartDashboard.getNumber("LSpinP", .0005);
    // if (newLeftSpinP != kPLeft) {
    //   kPLeft = newLeftSpinP;
    //   leftShooterPIDController.setP(kPLeft);
    // }
    // SmartDashboard.putNumber("LSpinP", kPLeft);

    // double newLeftSpinFF = SmartDashboard.getNumber("LSpinFF", .00016);
    // if (newLeftSpinFF != kFFLeft) {
    //   kFFLeft = newLeftSpinFF;
    //   leftShooterPIDController.setFF(kFFLeft);
    // }
    // SmartDashboard.putNumber("LSpinFF", kFFLeft);

    // double newRightSpinP = SmartDashboard.getNumber("RSpinP", .0005);
    // if (newRightSpinP != kPRight) {
    //   kPRight = newRightSpinP;
    //   rightShooterPIDController.setP(kPRight);
    // }
    // SmartDashboard.putNumber("RSpinP", kPRight);

    // double newRightSpinFF = SmartDashboard.getNumber("RSpinFF", .00016);
    // if (newRightSpinFF != kFFRight) {
    //   kFFRight = newRightSpinFF;
    //   rightShooterPIDController.setFF(kFFRight);
    // }
    // SmartDashboard.putNumber("RSpinFF", kFFRight);

    // SmartDashboard.putNumber("AnglekP", angleP);
    // trapAngle = SmartDashboard.getNumber("trap angle", 48);
    // SmartDashboard.putNumber("trap angle", trapAngle);
    // SmartDashboard.putNumber("aNGLe dest", trapAngle);

    //   double newAnglekD = SmartDashboard.getNumber("AnglekD", 0);
    //   if(newAnglekD!=angleD){
    //     angleD=newAnglekD;
    //     anglePIDController.setD(angleD);
    //   }
    //   SmartDashboard.putNumber("AnglekD", angleD);

    //  double newAngleI = SmartDashboard.getNumber("AnglekI", 0);
    //   if(newAngleI!=angleI){
    //     angleI=newAngleI;
    //     anglePIDController.setI(angleI);
    //   }
    //   SmartDashboard.putNumber("AnglekI", angleI);
  }
}
