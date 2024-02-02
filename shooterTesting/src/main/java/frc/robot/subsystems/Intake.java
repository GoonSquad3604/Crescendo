// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

 
 public boolean isToggled = false;
 private static Intake _instance; 


  public Intake() {


    
  }
 public static Intake getInstance() {
   if (_instance == null){
          _instance = new Intake();
   }
   return _instance;
 }



    

    



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
