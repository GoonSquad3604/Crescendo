// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class SwerveDefaultDrive extends Command {

  private SwerveDrive s_Swerve;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private BooleanSupplier robotCentricSup;
  private DoubleSupplier slower;

  private double speed;

  /** Creates a new SwerveDefaultDrive. */
  public SwerveDefaultDrive(
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      DoubleSupplier slower,
      BooleanSupplier robotCentricSup) {

    this.s_Swerve = SwerveDrive.getInstance();

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;
    this.slower = slower;

    addRequirements(s_Swerve);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (slower.getAsDouble() > .5) {
      speed = .25;
    } else speed = 1;
    /* Get Values, Deadband*/
    double translationVal =
        MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.General.stickDeadband)
            * speed;
    double strafeVal =
        MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.General.stickDeadband) * speed;
    double rotationVal =
        MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.General.stickDeadband) * speed;

    /* Drive */
    s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
        rotationVal * Constants.Swerve.maxAngularVelocity,
        !robotCentricSup.getAsBoolean(),
        true);
  }
}
