// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.DefaultDrive;
import frc.robot.subsystems.CANDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class RobotContainer {

  private final CANDrivetrain m_drivetrain = CANDrivetrain.getInstance();

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandJoystick operator = new CommandJoystick(1);
  
  private final Shooter s_Shooter = Shooter.getInstance();
  private final Indexer s_Indexer = Indexer.getInstance();
  private final Intake s_Intake = Intake.getInstance();

  private final SendableChooser<Command> autoChooser;


  public RobotContainer() {
    NamedCommands.registerCommand(
      "revShooter", new InstantCommand(() -> s_Shooter.runShooter()));
    NamedCommands.registerCommand(
      "stopShooter", new InstantCommand(() -> s_Shooter.stopShooter()));
    NamedCommands.registerCommand(
      "runIndexer", new InstantCommand(() -> s_Indexer.runIndexer()));
    NamedCommands.registerCommand(
      "runIntake", new InstantCommand(() -> s_Intake.setIntakeSpeed(0.6)));
    NamedCommands.registerCommand(
      "hingeDown", new InstantCommand(() -> s_Intake.setHingeTo(-0.27)));

    m_drivetrain.setDefaultCommand(new DefaultDrive(() -> driver.getLeftY(), () -> driver.getRightX()));

    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  private void configureBindings() {
  
    m_drivetrain.arcadeDrive(-driver.getLeftY(), -driver.getRightX());

    driver.y().onTrue(new InstantCommand(() -> s_Intake.raiseHinge()));
    driver.y().onFalse(new InstantCommand(() -> s_Intake.stopHinge()));

    driver.a().onTrue(new InstantCommand(() -> s_Intake.lowerHinge()));
    driver.a().onFalse(new InstantCommand(() -> s_Intake.stopHinge()));


    operator.button(6).onTrue(new ParallelCommandGroup(
      new InstantCommand(() -> s_Intake.setHingeTo(-0.27)),
        new InstantCommand(() -> s_Indexer.runIndexer()), 
          new InstantCommand(() -> s_Intake.setIntakeSpeed(0.6))));
    operator.button(6).onFalse(new ParallelCommandGroup(
      new InstantCommand(() -> s_Intake.setHingeTo(0)),
        new InstantCommand(() -> s_Indexer.stopIndexer()), 
          new InstantCommand(() -> s_Intake.setIntakeSpeed(0))));

    operator.button(7).onTrue(new ParallelCommandGroup(
      new InstantCommand(() -> s_Shooter.runShooterReverse()),
        new InstantCommand(() -> s_Indexer.runIndexerReverse())));
    operator.button(7).onFalse(new ParallelCommandGroup(
      new InstantCommand(() -> s_Shooter.stopShooter()),
        new InstantCommand(() -> s_Indexer.stopIndexer())));

    operator.button(9).onTrue(new InstantCommand(() -> s_Shooter.runShooter()));

    operator.button(10).onTrue(new InstantCommand(() -> s_Shooter.spinTurretCounterClockwise()));
    operator.button(10).onFalse(new InstantCommand(() -> s_Shooter.stopTurret()));

    operator.button(11).onTrue(new InstantCommand(() -> s_Shooter.spinTurretClockwise()));
    operator.button(11).onFalse(new InstantCommand(() -> s_Shooter.stopTurret()));

    operator.button(12).onTrue(new InstantCommand(() -> s_Indexer.runIndexer()));
    operator.button(12).onFalse(new ParallelCommandGroup(
      new InstantCommand(() -> s_Indexer.stopIndexer()),
        new InstantCommand(() -> s_Shooter.stopShooter())));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
