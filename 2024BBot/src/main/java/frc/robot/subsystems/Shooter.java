// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

  
  package frc.robot.subsystems;
  
  import edu.wpi.first.hal.CANAPITypes.CANDeviceType;
  import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
  import edu.wpi.first.wpilibj2.command.SubsystemBase;
  import frc.robot.Constants;
  
  import com.revrobotics.CANSparkFlex;
  import com.revrobotics.RelativeEncoder;
  import com.revrobotics.SparkPIDController;
  import com.revrobotics.CANSparkBase.ControlType;
  import com.revrobotics.CANSparkLowLevel.MotorType;
  
  public class Shooter extends SubsystemBase {
  //Declares variables
    private CANSparkFlex leftShooterMotor;
    private CANSparkFlex rightShooterMotor;
    private CANSparkFlex indexMotor;
    private CANSparkFlex angleMotor;  

    private RelativeEncoder leftShooterEncoder;
    private RelativeEncoder rightShooterEncoder;
    private RelativeEncoder indexEncoder;
    private RelativeEncoder angleEncoder;

    private SparkPIDController leftShooterPIDController;
    private SparkPIDController rightShooterPIDController;
    private SparkPIDController IndexPIDController;
    private SparkPIDController anglePIDController;
  
    private DigitalInput sensor;

    private static Shooter _instance;
  
    private double anglePos;
    private int leftRPM;
    private int rightRPM;
    private int IndexRPM;

    /** Creates a new Shooter. */
    public Shooter() {
    
        leftShooterMotor = new CANSparkFlex(Constants.ShooterConstants.leftID, MotorType.kBrushless);
        leftShooterEncoder = leftShooterMotor.getEncoder();
        leftShooterPIDController = leftShooterMotor.getPIDController();
  
        rightShooterMotor = new CANSparkFlex(Constants.ShooterConstants.rightID, MotorType.kBrushless);
        rightShooterEncoder = rightShooterMotor.getEncoder();
        rightShooterPIDController = rightShooterMotor.getPIDController();

        angleMotor = new CANSparkFlex(Constants.ShooterConstants.angleID, MotorType.kBrushless);
        angleEncoder = angleMotor.getEncoder();
        anglePIDController = angleMotor.getPIDController();

        indexMotor = new CANSparkFlex(Constants.ShooterConstants.indexID, MotorType.kBrushless);
        indexEncoder = indexMotor.getEncoder();
        IndexPIDController = indexMotor.getPIDController();

        leftShooterMotor.restoreFactoryDefaults();
        rightShooterMotor.restoreFactoryDefaults();
        angleMotor.restoreFactoryDefaults();
        indexMotor.restoreFactoryDefaults();
  
        //PIDS
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

        IndexPIDController.setP(Constants.ShooterConstants.indexkP);
        IndexPIDController.setI(Constants.ShooterConstants.indexkI);
        IndexPIDController.setD(Constants.ShooterConstants.indexkD);
        IndexPIDController.setFF(Constants.ShooterConstants.indexkF);


        rightShooterPIDController.setOutputRange(-1, 1);
        leftShooterPIDController.setOutputRange(-1, 1);
        anglePIDController.setOutputRange(-1, 1);
        IndexPIDController.setOutputRange(-1, 1);
        //Inverts left shooter motor
        leftShooterMotor.setInverted(true);
    }
  
   public static Shooter getInstance() {
        if(_instance == null){
          _instance = new Shooter();
        }
        return _instance;
   }
 
  public void setPower(double speed) {
    //Sets motor speed
    leftShooterMotor.set(speed);
    rightShooterMotor.set(speed);
  }

  public void setAngle(){
    anglePIDController.setReference(anglePos, ControlType.kPosition);
  }

  public void setRPM(){
    leftShooterPIDController.setReference(leftRPM, ControlType.kVelocity);
    rightShooterPIDController.setReference(rightRPM, ControlType.kVelocity);
  }
  public void stop(){
    leftShooterMotor.set(0);
    rightShooterMotor.set(0);
  }

  public void setIndexRPM(){
    IndexPIDController.setReference(IndexRPM, ControlType.kVelocity);
  }

  public void setIndexPower(double speed){
    indexMotor.set(speed);
  }

  public void indexStop(){
  indexMotor.set(0);
  }

  public boolean hasNote(){
    return !sensor.get();
  }
  
    @Override
  public void periodic() {
    SmartDashboard.putBoolean("sensor has target: ", hasNote());

    }
  }
  

