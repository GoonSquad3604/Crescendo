// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;


public class SwerveDefaultDrive extends Command {
  /** Creates a new SwerveDefaultDrive. */

  private SwerveDrive s_Swerve;    
  private DoubleSupplier xSupplier;
  private DoubleSupplier ySupplier;
  private DoubleSupplier rotSupplier;
  private BooleanSupplier rotLocatSupplier;
  private BooleanSupplier rotLocat2Supplier;
  private DoubleSupplier slower;
  private double speed = 1.0;

  public SwerveDefaultDrive(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotSupplier, BooleanSupplier rotLocSupplier, BooleanSupplier rotLocSupplier2, DoubleSupplier slower) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Swerve = SwerveDrive.getInstance();

    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.rotSupplier = rotSupplier;
    this.rotLocatSupplier = rotLocSupplier;
    this.rotLocat2Supplier = rotLocSupplier2;
    this.slower = slower;

    addRequirements(s_Swerve);


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(slower.getAsDouble() > .5) {
      speed = 0.25;
    }
    else speed = 1;

    double translationVal = MathUtil.applyDeadband(xSupplier.getAsDouble(), Constants.General.deadband)*speed;
    double strafeVal = MathUtil.applyDeadband(ySupplier.getAsDouble(), Constants.General.deadband)*speed;
    double rotationVal = MathUtil.applyDeadband(rotSupplier.getAsDouble(), Constants.General.deadband)*speed;

    /* Drive */
    s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
        rotationVal * Constants.Swerve.maxAngularVelocity,  
        true,
        rotLocatSupplier.getAsBoolean(),
        rotLocat2Supplier.getAsBoolean()
    );
  }

}
