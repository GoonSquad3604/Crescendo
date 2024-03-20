// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class rAMP extends SubsystemBase {
  private static rAMP instance;
  private AbsoluteEncoder rAMPEncoder;
  private CANSparkMax rAMPSparkMax;
  private double rAMPP, rAMPI, rAMPD;
  private SparkPIDController rAMPPIDController;

  public rAMP() {
    rAMPEncoder = rAMPSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    rAMPPIDController = rAMPSparkMax.getPIDController();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
