// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private CANSparkMax climbMotor;
  public Climber() {
    // climbMotor = new CANSparkMax(17,mo );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
