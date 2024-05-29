// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private static Intake _instance;
  private WPI_TalonSRX bigWheel;
  private WPI_TalonSRX hinge;
  private WPI_TalonSRX mainIntake;
  private PIDController hingePID;

  private DutyCycleEncoder hingeEncoder;
  private double hingeTop = 0.12;
  private double hingeBottom = -0.16;

  
  public Intake() {

    bigWheel = new WPI_TalonSRX(9);
    hinge = new WPI_TalonSRX(10);
    hinge.setNeutralMode(NeutralMode.Brake);
    mainIntake = new WPI_TalonSRX(11);

    hingeEncoder = new DutyCycleEncoder(0);
    hingeEncoder.reset();

    hingePID = new PIDController(1, 0, 0);
        
  }

   /** Creates a new Intake. */
  public static Intake getInstance(){
    if (_instance == null){
      _instance = new Intake();
    }
    return _instance;
  }

  public void runIntake(){
     bigWheel.set(0.3);
     mainIntake.set(-0.3);

    
  }

  public void stopIntake(){
    bigWheel.set(0);
    mainIntake.set(0);
  }

  public void raiseHinge(){
    hinge.set(0.3);
  }

  public void lowerHinge(){
    hinge.set(-0.3);
  }

  public void stopHinge(){
    hinge.set(0);
  }

  public void setHingeTo(double position){
    hinge.set(hingePID.calculate(hingeEncoder.getDistance(), position));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("encoder pos", hingeEncoder.getDistance());
    SmartDashboard.putNumber("calculated number", hingePID.calculate(hingeEncoder.getDistance(), -.26));
  }
}
