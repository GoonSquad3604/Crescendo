// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.shooter.FeedUntillSensor;
import frc.robot.commands.shooter.RepositionNote;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.StateController;

public class RobotContainer {
  private double MaxSpeed =
      TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate =
      1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driver = new CommandXboxController(0); // My joystick
  private final CommandJoystick buttonBox = new CommandJoystick(1);
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final Shooter s_Shooter = Shooter.getInstance();
  private final Intake s_Intake = Intake.getInstance();
  private final Climber s_Climber = Climber.getInstance();
  private final StateController s_StateController = StateController.getInstance();
  //   private final Vision s_Vision = Vision.getInstance();
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
  // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);
  private final SendableChooser<Command> autoChooser;

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with
                    // negative Y (forward)
                    .withVelocityY(
                        -driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        -driver.getRightX()
                            * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));
    Trigger homeTrigger = new Trigger(s_StateController::isHomeMode);
    Trigger indexTrigger = new Trigger(s_Shooter::hasNote);
    Trigger climberTrigger = new Trigger(s_StateController::isClimberMode);
    Trigger ampTrigger = new Trigger(s_StateController::isAmpMode);
    Trigger speakerTrigger = new Trigger(s_StateController::isSpeakerMode);
    driver
        .rightTrigger()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    drive
                        .withVelocityX(-driver.getLeftY() * MaxSpeed * .1)
                        .withVelocityY(-driver.getLeftX() * MaxSpeed * .1)
                        .withRotationalRate(-driver.getRightX() * MaxAngularRate * .5)));
    driver.leftTrigger().whileTrue(drivetrain.applyRequest(() -> brake));
    driver
        .b()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

    // reset the field-centric heading on left bumper press
    driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    // buttonBox.button(1).onTrue(new InstantCommand(() ->s_Intake.raiseHinge()));
    // buttonBox.button(1).onFalse(new InstantCommand(() -> s_Intake.stopHinge()));
    // buttonBox.button(2).onTrue(new InstantCommand(() -> s_Intake.lowerHinge()));
    // buttonBox.button(2).onFalse(new InstantCommand(() -> s_Intake.stopHinge()));
    // buttonBox.button(3).onTrue(new InstantCommand(() -> s_Intake.runIntake()));
    // buttonBox.button(3).onFalse(new InstantCommand(() -> s_Intake.stopIntake()));

    // buttonBox.button(4).onTrue(new InstantCommand(() -> s_Shooter.raiseAngle()));
    // buttonBox.button(4).onFalse(new InstantCommand(() -> s_Shooter.stopAngle()));
    // buttonBox.button(5).onTrue(new InstantCommand(() -> s_Shooter.lowerAngle()));
    // buttonBox.button(5).onFalse(new InstantCommand(() -> s_Shooter.stopAngle()));
    // buttonBox.button(6).onTrue(new InstantCommand(() -> s_Shooter.setPower(.2)));
    // buttonBox.button(6).onFalse(new InstantCommand(() -> s_Shooter.stopShooter()));
    // buttonBox.button(7).onTrue(new InstantCommand(() -> s_Shooter.setIndexPower(.3)));
    // buttonBox.button(7).onFalse(new InstantCommand(() -> s_Shooter.indexStop()));

    // buttonBox.button(8).onTrue(new InstantCommand(() -> s_Climber.climberUp()));
    // buttonBox.button(8).onFalse(new InstantCommand(() -> s_Climber.stopClimber()));

    // buttonBox.button(9).onTrue(new InstantCommand(() -> s_Climber.climberDown()));
    // buttonBox.button(9).onFalse(new InstantCommand(() -> s_Climber.stopClimber()));

    buttonBox
        .button(1)
        .onTrue(
            new ParallelCommandGroup(
                new InstantCommand(
                    () -> s_Intake.setHingeTo(Constants.IntakeConstants.hingeUp), s_Intake),
                new InstantCommand(() -> s_StateController.setTravel(), s_StateController),
                new InstantCommand(()-> s_Shooter.shooterTo(s_StateController.getAngle()),s_Shooter)));
    buttonBox
        .button(2)
        .onTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> s_StateController.setSpeaker(), s_StateController),
                new InstantCommand(
                    () -> s_Intake.setHingeTo(Constants.IntakeConstants.hingeUp), s_Intake),
                new InstantCommand(
                    () -> s_Shooter.shooterTo(Constants.ShooterConstants.shooterSpeaker))));
    buttonBox
        .button(3)
        .onTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> s_StateController.setAmp(), s_StateController),
                new InstantCommand(
                    () -> s_Intake.setHingeTo(Constants.IntakeConstants.hingeUp), s_Intake),
                new InstantCommand(
                    () -> s_Shooter.shooterTo(Constants.ShooterConstants.shooterAmp))));
    buttonBox
        .button(4)
        .onTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> s_StateController.setTrap(), s_StateController),
                new InstantCommand(
                    () -> s_Intake.setHingeTo(Constants.IntakeConstants.hingeUp), s_Intake),
                new InstantCommand(
                    () -> s_Shooter.shooterTo(Constants.ShooterConstants.shooterTrap))));

    buttonBox
        .button(5)
        .onTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> s_StateController.setClimber(), s_StateController),
                new InstantCommand(
                    () -> s_Intake.setHingeTo(Constants.IntakeConstants.hingeUp), s_Intake)));

    buttonBox
        .button(6)
        .onTrue(
            new ParallelCommandGroup(
                new InstantCommand(
                    () -> s_Intake.setHingeTo(Constants.IntakeConstants.hingeDown), s_Intake),
                new SequentialCommandGroup(
                    new InstantCommand(() -> s_StateController.setHome(), s_StateController),
                    new InstantCommand(
                        () -> s_Shooter.shooterTo(s_StateController.getAngle()), s_Shooter))));

    buttonBox
        .button(7)
        .and(homeTrigger)
        .onTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> s_Intake.runIntake()),
                new SequentialCommandGroup(new FeedUntillSensor(), new RepositionNote())));

    buttonBox
        .button(7)
        .and(indexTrigger)
        .onTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> s_Intake.stopIntake()),
                new InstantCommand(() -> s_Shooter.stopShooter())));

    buttonBox
        .button(7)
        .onFalse(
            new ParallelCommandGroup(
                new InstantCommand(() -> s_Intake.stopIntake()),
                new InstantCommand(() -> s_Shooter.indexStop(), s_Shooter)));

    buttonBox.button(8).onTrue(new InstantCommand(() -> s_Intake.vomit()));
    buttonBox.button(8).onTrue(new InstantCommand(() -> s_Shooter.setIndexPower(-.3)));
    buttonBox.button(8).onFalse(new InstantCommand(() -> s_Intake.stopIntake()));
    buttonBox.button(8).onFalse(new InstantCommand(() -> s_Shooter.indexStop()));
    buttonBox
        .button(9)
        .onTrue(
            new InstantCommand(
                () ->
                    s_Shooter.setShooterRPM(
                        s_StateController.getLeftShooterSpeed(),
                        s_StateController.getRightShooterSpeed()),
                s_Shooter));
    buttonBox
        .button(9)
        .and(climberTrigger)
        .onTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> s_Shooter.stopShooter(), s_Shooter),
                new InstantCommand(() -> s_Shooter.indexStop())));

    buttonBox
        .button(10)
        .and(speakerTrigger)
        .onTrue(new InstantCommand(() -> s_Shooter.setShooterRPMSpeaker()));
    // buttonBox.button(10).onTrue(new InstantCommand(() -> s_Shooter.setIndexPower(-.2)));
    // buttonBox.button(10).onFalse(new InstantCommand(()-> s_Shooter.indexStop()));
    buttonBox.button(10).onFalse(new InstantCommand(() -> s_Shooter.stopShooter()));
    buttonBox
        .button(10)
        .and(climberTrigger)
        .onTrue(new InstantCommand(() -> s_Climber.raiseCimber()));

    buttonBox
        .button(11)
        .and(climberTrigger)
        .onTrue(new InstantCommand(() -> s_Shooter.stopAngle(), s_Shooter));
    buttonBox
        .button(11)
        .and(climberTrigger)
        .onTrue(new InstantCommand(() -> s_Climber.lowerClimber()));
    buttonBox
        .button(11)
        .and(speakerTrigger)
        .onTrue(new InstantCommand(() -> s_Shooter.shooterTo(33)));
    buttonBox
        .button(12)
        .onTrue(new InstantCommand(() -> s_Shooter.setIndexRPM(s_StateController.getIndexSpeed())));

    // buttonBox.button(12).and(ampTrigger).onTrue(new ShootAmp());

    buttonBox
        .button(12)
        .and(climberTrigger)
        .onTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> s_Shooter.stopShooter(), s_Shooter),
                new InstantCommand(() -> s_Shooter.indexStop())));
    buttonBox
        .button(12)
        .onFalse(
            new ParallelCommandGroup(
                new InstantCommand(() -> s_Shooter.stopShooter(), s_Shooter),
                new InstantCommand(() -> s_Shooter.indexStop())));

    driver.a().onTrue(new InstantCommand(() -> s_Intake.cleam()));
    driver.a().onFalse(new InstantCommand(() -> s_Intake.stopIntake()));
    driver.x().onTrue(new InstantCommand(() -> s_Climber.climberUp()));
    driver.x().onFalse(new InstantCommand(() -> s_Climber.stopClimber()));
    driver.start().onTrue(new InstantCommand(() -> s_Climber.climberDown()));
    driver.start().onFalse(new InstantCommand(() -> s_Climber.stopClimber()));
    driver.y().and(indexTrigger.negate()).onTrue(
        new ParallelCommandGroup(
            new InstantCommand(()-> s_Shooter.setShooterRPM(200, -200)),
            new InstantCommand(()-> s_Shooter.setIndexPower(.2)))
    );
    driver.y().and(indexTrigger.negate()).onFalse(
        new ParallelCommandGroup(
            new InstantCommand(()-> s_Shooter.stopShooter()),
            new InstantCommand(() ->s_Shooter.indexStop())
        )
    );
    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    NamedCommands.registerCommand(
        "runIntake",
        new SequentialCommandGroup(
            new InstantCommand(() -> s_Intake.runIntake(), s_Intake),
            new FeedUntillSensor(),
            new RepositionNote()));
    NamedCommands.registerCommand(
        "stopShooter", new InstantCommand(() -> s_Shooter.stopShooter(), s_Shooter));
    NamedCommands.registerCommand(
        "intakeDown",
        new InstantCommand(
            () -> s_Intake.setHingeTo(Constants.IntakeConstants.hingeDown), s_Intake));
    NamedCommands.registerCommand(
        "intakeUp",
        new InstantCommand(() -> s_Intake.setHingeTo(Constants.IntakeConstants.hingeUp)));
    NamedCommands.registerCommand(
        "shooterTo", new InstantCommand(() -> s_Shooter.shooterTo(20), s_Shooter));
    NamedCommands.registerCommand(
        "revShooter", new InstantCommand(() -> s_Shooter.setShooterRPM(-4500, 6000), s_Shooter));
    //    NamedCommands.registerCommand( "revShooter", Commands.print("marker1"));
    NamedCommands.registerCommand("fire", new InstantCommand(() -> s_Shooter.setIndexRPM(-6000)));
    NamedCommands.registerCommand("stopIntake", new InstantCommand(() -> s_Intake.stopIntake()));
    NamedCommands.registerCommand(
        "runIndex", new InstantCommand(() -> s_Shooter.setIndexPower(-.4)));
    NamedCommands.registerCommand(
        "stopIndex", new InstantCommand(() -> s_Shooter.indexStop(), s_Shooter));
    NamedCommands.registerCommand(
        "shooterHome", new InstantCommand(() -> s_Shooter.shooterTo(65), s_Shooter));
    NamedCommands.registerCommand(
        "shooterTo1", new InstantCommand(() -> s_Shooter.shooterTo(32), s_Shooter));
    NamedCommands.registerCommand(
        "shooterTo2", new InstantCommand(() -> s_Shooter.shooterTo(30), s_Shooter));
    NamedCommands.registerCommand(
        "revShooter1", new InstantCommand(() -> s_Shooter.setShooterRPM(-3500, 4000), s_Shooter));
    NamedCommands.registerCommand(
        "shooterSpeaker", new InstantCommand(() -> s_Shooter.shooterTo(56), s_Shooter));
        NamedCommands.registerCommand(
        "shooterTo3", new InstantCommand(() -> s_Shooter.shooterTo(27), s_Shooter));
    configureBindings();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
