// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private static Intake _instance;
  private WPI_TalonSRX bigWheel;
  private WPI_TalonSRX hindge;
  private WPI_TalonSRX mainIntake;

 
  public Intake() {
     bigWheel = new WPI_TalonSRX(9);
     hindge = new WPI_TalonSRX(10);
     mainIntake = new WPI_TalonSRX(11);


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
    hindge.set(0.3);
  }
  public void lowerHinge(){
    hindge.set(-0.3);
  }
  public void stopHinge(){
    hindge.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
