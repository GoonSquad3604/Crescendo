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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.util.LookUpTable;
import org.littletonrobotics.junction.AutoLogOutput;

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
  @AutoLogOutput private double leftRPM = -6000;
  @AutoLogOutput private double rightRPM = 6000;
  

  @AutoLogOutput private double trackedAngle = 56;

  @AutoLogOutput private double trackedLookUpTableAngle = 56;

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
    angleEncoder.setInverted(true);

    rightShooterEncoder = rightShooterMotor.getEncoder();
    rightShooterPIDController = rightShooterMotor.getPIDController();

    leftShooterEncoder = leftShooterMotor.getEncoder();
    leftShooterPIDController = leftShooterMotor.getPIDController();

    // PIDS

    anglePIDController.setFeedbackDevice(angleEncoder);

    // anglePIDController.setI(Constants.ShooterConstants.anglekI);
    // anglePIDController.setD(Constants.ShooterConstants.anglekD);

    anglePIDController.setP(Constants.ShooterConstants.anglekP);
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

    rightShooterPIDController.setOutputRange(-1, 1);
    leftShooterPIDController.setOutputRange(-1, 1);
    anglePIDController.setOutputRange(-.3, .3);

    leftShooterMotor.enableVoltageCompensation(12);
    rightShooterMotor.enableVoltageCompensation(12);
    // leftShooterMotor.setIdleMode(IdleMode.kBrake);
    // rightShooterMotor.setIdleMode(IdleMode.kBrake);

    angleMotor.setIdleMode(IdleMode.kBrake);
  }

  public static Shooter getInstance() {
    if (_instance == null) {
      _instance = new Shooter();
    }
    return _instance;
  }

  public void setPower(double power) {
    // Sets motor power
    leftShooterMotor.set(-power);
    rightShooterMotor.set(power);
  }
  //Sets speed of the shooter when firing at the speaker
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
  //Sets speed of the shooter when firing at the amp
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
  //Sets shooter to aspecific speed
  public void setShooterRPM(double leftRPM, double rightRPM) {
    leftShooterPIDController.setReference(leftRPM, ControlType.kVelocity);
    rightShooterPIDController.setReference(rightRPM, ControlType.kVelocity);
  }

  public void babyBird(double leftRPM, double rightRPM) {
    leftShooterPIDController.setReference(leftRPM, ControlType.kVelocity);
    rightShooterPIDController.setReference(-rightRPM, ControlType.kVelocity);
  }
  //Look up the definition of stop
  public void stopShooter() {
    leftShooterMotor.set(0);
    rightShooterMotor.set(0);
  }
//Like the above method, but uses speed instead of power
  public void stopShooterRPM() {
    leftShooterPIDController.setReference(0, ControlType.kVelocity);
    rightShooterPIDController.setReference(-0, ControlType.kVelocity);
  }

  // ShooterAngle methods:
  //Increases the angle of the shooter
  public void raiseAngle() {
    angleMotor.set(.2);
  }
  //lowers the angle of the shooter
  public void lowerAngle() {
    angleMotor.set(-.2);
  }
  //Halt
  public void stopAngle() {
    angleMotor.set(.00);
  }
  //returns the position of the angle encoder
  public double getShooterAngleClicks() {
    return angleEncoder.getPosition();
  }
//Changes P value for when the shooter goes up
  public void setUpP() {
    anglePIDController.setP(Constants.ShooterConstants.angleUpP);
  }
//Changes P value for when the shooter goes down
  public void setDownP() {
    anglePIDController.setP(Constants.ShooterConstants.angleDownP);
  }

  // Indexer methods

  // -0.002667*X + 0.7915
  // public void shooterTo(double position) {
  //   anglePIDController.setReference(position * .00251 + .6471, ControlType.kPosition);
  // }

  // .004324 + .5256
  public void shooterTo() {
    anglePIDController.setReference(Constants.ShooterConstants.trapAngle * .00297 + .49717, ControlType.kPosition);
  }
//Sets shooter angle to specific position
  public void shooterToAngle(double angle) {
    anglePIDController.setReference((angle - 131.8) / -372.2, ControlType.kPosition);
  }
//returs array of speeds from each shooter encoder.
  public double[] getRPMS() {
    double[] rpms = {leftShooterEncoder.getVelocity(), rightShooterEncoder.getVelocity()};
    return rpms;
  }
//returns angle to speaker using the robot's pose 
  

   
  @Override
  public void periodic() {

    // trackedAngle = (getShooterAngleClicks() - .6471) / .00251;

    SmartDashboard.putNumber("ShooterAngleEncoder", getShooterAngleClicks());
    SmartDashboard.putNumber("ShooterAngle", ((getShooterAngleClicks() * -372.2) + 131.8));
    leftRPM = leftShooterEncoder.getVelocity();
    rightRPM = rightShooterEncoder.getVelocity();
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
    //trapAngle = SmartDashboard.getNumber("trap angle", 48);
   
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
