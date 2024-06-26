// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.CANDrivetrain;
import frc.robot.subsystems.Indexer;

public class RobotContainer {

  private final CANDrivetrain m_drivetrain = CANDrivetrain.getInstance();

  private final CommandXboxController driver = new CommandXboxController(0);
  
  private final Turret s_Turret = Turret.getInstance();
  private final Indexer s_Indexer = Indexer.getInstance();



  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {

  
    m_drivetrain.arcadeDrive(-driver.getLeftY(), -driver.getRightX());


    driver.a().onTrue(new InstantCommand(() -> s_Turret.spinTurretCounterClockwise()));
    driver.a().onFalse(new InstantCommand(() -> s_Turret.stopTurret()));

    driver.b().onTrue(new InstantCommand(() -> s_Turret.spinTurretClockwise()));
    driver.b().onFalse(new InstantCommand(() -> s_Turret.stopTurret()));

    driver.rightTrigger().onTrue(new InstantCommand(() -> s_Turret.runShooter()));
    driver.rightTrigger().onFalse(new InstantCommand(() -> s_Turret.stopShooter()));

    driver.leftTrigger().onTrue(new InstantCommand(() -> s_Indexer.runIndexer()));
    driver.leftTrigger().onFalse(new InstantCommand(() -> s_Indexer.stopIndexer()));

    driver.leftBumper().onTrue(new ParallelCommandGroup(new InstantCommand(() -> s_Turret.runShooterReverse()),
      new InstantCommand(() -> s_Indexer.runIndexerReverse())));

    driver.leftBumper().onFalse(new ParallelCommandGroup(new InstantCommand(() -> s_Turret.stopShooter()),
      new InstantCommand(() -> s_Indexer.stopIndexer())));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
