// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import java.util.function.DoubleSupplier;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.subsystems.CANDrivetrain;

public class DefaultDrive extends Command {
  /** Creates a new DefaultDrive. */
  private CANDrivetrain s_DriveTrain;

  private DoubleSupplier leftDriveSupplier;
  private DoubleSupplier rightDriveSupplier;
  
  private double speed = Constants.General.robotSpeed;


  public DefaultDrive(DoubleSupplier leftDriveSupplier, DoubleSupplier rightDriveSupplier) {
    s_DriveTrain = CANDrivetrain.getInstance();
    
    this.leftDriveSupplier = leftDriveSupplier;
    this.rightDriveSupplier = rightDriveSupplier;
    

    addRequirements(s_DriveTrain);
  }

 
@Override
  public void execute() {
    double leftVal = MathUtil.applyDeadband(leftDriveSupplier.getAsDouble(), Constants.General.deadband)*speed;
  
    double rightVal = MathUtil.applyDeadband(rightDriveSupplier.getAsDouble(), Constants.General.deadband)*speed;
  
    s_DriveTrain.arcadeDrive(leftVal, rightVal);
  
  }
  
}