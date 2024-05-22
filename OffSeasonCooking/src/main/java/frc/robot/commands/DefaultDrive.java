// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import java.util.function.DoubleSupplier;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.CANDrivetrain;

public class DefaultDrive extends Command {
  /** Creates a new DefaultDrive. */
  private CANDrivetrain s_DriveTrain;

  private DoubleSupplier leftDriveSupplier;
  private DoubleSupplier rightDriveSupplier;
  
  private double speed = 1.0;


  public DefaultDrive(DoubleSupplier leftDriveSupplier, DoubleSupplier rightDriveSupplier) {
    s_DriveTrain = CANDrivetrain.getInstance();
    
    this.leftDriveSupplier = leftDriveSupplier;
    this.rightDriveSupplier = rightDriveSupplier;
    

    addRequirements(s_DriveTrain);
  }

  //Called every time the scheduler runs while the command is scheduled.
 
@Override
  public void execute() {
  //   if(slower.getAsDouble() > .5) {
  //     speed = 0.25;
  //   }
  //   else speed = 1;
    double leftVal = MathUtil.applyDeadband(leftDriveSupplier.getAsDouble(), 0.1)*speed;
  
    double rightVal = MathUtil.applyDeadband(rightDriveSupplier.getAsDouble(), 0.1)*speed;
  
  // s_DriveTrain.setLeftDrivepower(leftVal);

  // s_DriveTrain.setRightDrivepower(rightVal);
    s_DriveTrain.arcadeDrive(leftVal, rightVal);
  
  }
  
}