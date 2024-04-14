// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.LED.RAINBOW;
import frc.robot.commands.LED.SetLEDSYellow;
import frc.robot.commands.LED.SpeakerLEDMode;
import frc.robot.commands.climber.MagicClimb;
import frc.robot.commands.intake.SetIntakeDown;
import frc.robot.commands.shooter.AmpFireNew;
import frc.robot.commands.shooter.FeedUntillSensor;
import frc.robot.commands.shooter.RepositionNote;
import frc.robot.commands.shooter.RepositionNoteAuto;
import frc.robot.commands.stateController.AmpMode;
import frc.robot.commands.stateController.ClimberMode;
import frc.robot.commands.stateController.SpeakerMode;
import frc.robot.commands.stateController.TrapMode;
import frc.robot.commands.stateController.TravelMode;
import frc.robot.commands.vision.AutoShootAngleNew;
import frc.robot.commands.vision.LookUpTableAuton;
import frc.robot.commands.vision.LookUpTableInst;
import frc.robot.commands.vision.RotateToAmp;
import frc.robot.commands.vision.RotateToSpeaker;
// import frc.robot.commands.vision.Aim;
// import frc.robot.commands.vision.AimPID;
// import frc.robot.commands.vision.BetterAim;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Flipper;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.StateController;
import frc.robot.subsystems.Vision;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

public class RobotContainer {
  private double MaxSpeed =
      TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 3 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController pit = new CommandXboxController(2); // My joystick
  // 69

  private final CommandXboxController driver = new CommandXboxController(0); // My joystick
  private final CommandJoystick buttonBox = new CommandJoystick(1);
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final LED m_LED =  new LED(1, Constants.LEDConstants.lengthLeft);
//   private final LED rightLED = new LED(0, Constants.LEDConstants.lengthRight);
  private final Shooter s_Shooter = Shooter.getInstance();
  private final Intake s_Intake = Intake.getInstance();
  private final Climber s_Climber = Climber.getInstance();
  private final Flipper s_Flipper = Flipper.getInstance();
  private final Index s_Index = Index.getInstance();
  private final StateController s_StateController = StateController.getInstance();

  private final Vision rightVision =
      new Vision("right", Constants.VisionConstants.RIGHT_ROBOT_TO_CAMERA);
  private final Vision leftVision =
      new Vision("left", Constants.VisionConstants.LEFT_ROBOT_TO_CAMERA);

  private Command aimAndShootCommand;
  private Command aimAndShootCommandAmp;

  //   private Command shimmy;
  //   private final Vision s_Vision = Vision.getInstance();
  public final SwerveRequest.FieldCentric drive =
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
    Trigger intakeTrigger = new Trigger(s_StateController::isIntakeMode);
    Trigger indexTrigger = new Trigger(s_Index::hasNote);
    Trigger climberTrigger = new Trigger(s_StateController::isClimberMode);
    Trigger ampTrigger = new Trigger(s_StateController::isAmpMode);
    Trigger speakerTrigger = new Trigger(s_StateController::isSpeakerMode);
    Trigger travelTrigger = new Trigger(s_StateController::isTravelMode);
    driver
        .rightTrigger()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    drive
                        .withVelocityX(-driver.getLeftY() * MaxSpeed * .2)
                        .withVelocityY(-driver.getLeftX() * MaxSpeed * .2)
                        .withRotationalRate(-driver.getRightX() * MaxAngularRate * .5)));
    // driver.leftTrigger().whileTrue(drivetrain.applyRequest(() -> brake));
    driver
        .b()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

    // reset the field-centric heading on left bumper press
    driver
        .leftBumper()
        .and(driver.rightBumper())
        .onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    // pit.leftBumper().onTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // pit.rightBumper().onTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // pit.rightTrigger().onTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    // pit.leftTrigger().onTrue(drivetrain.sysIdDynamic(Direction.kForward));

    // pit.a().onTrue(new InstantCommand(() -> s_Shooter.shooterTo(56)));
    // pit.a()
    //     .onTrue(
    //         new InstantCommand(() -> s_Intake.setHingeTo(Constants.IntakeConstants.hingeStart)));
    buttonBox.button(1).onTrue(new TravelMode(m_LED));
    buttonBox.button(2).onTrue(new SpeakerLEDMode(m_LED));
    buttonBox
        .button(2)
        .onTrue(
            new SpeakerMode()
                // .andThen(new InstantCommand(() ->
                // s_Shooter.lookUpTable(drivetrain.getState().Pose)))
                .andThen(new AutoShootAngleNew(s_Shooter, drivetrain))
            // .andThen(new ReturnDistanc(s_Shooter, drivetrain.getState().Pose))
            );
    // buttonBox.button(3).onTrue(new AmpMode().andThen(new RepositionForAmp()));
    buttonBox.button(3).onTrue(new AmpMode());
    buttonBox.button(3).onTrue(new SetLEDSYellow(m_LED));
    // buttonBox.button(3).onTrue(new InstantCommand(() -> s_Shooter.setShooterRPM(-1500,2000)));
    // buttonBox.button(3).onTrue(new InstantCommand(() -> s_Shooter.setPower(.2)));

    // buttonBox.button(3).onTrue(new RepositionForAmp());
    buttonBox.button(4).onTrue(new TrapMode());
    buttonBox.button(5).onTrue(new ClimberMode());
    buttonBox.button(5).onTrue(new RAINBOW(m_LED));
    // buttonBox.button(6).onTrue(new InstantCommand(() -> s_rAMP.setrAMPUp()));

    buttonBox
        .button(6)
        .onTrue(
            new SetIntakeDown()
                .andThen(
                    new ParallelCommandGroup(
                        new InstantCommand(() -> s_Intake.runIntake()),
                        new SequentialCommandGroup(
                            new FeedUntillSensor(m_LED, s_Intake),
                            new RepositionNote()))));
    buttonBox.button(6).onFalse(new TravelMode(m_LED));
    buttonBox.button(6).onFalse(new InstantCommand(() -> s_Intake.stopIntake()));

    // buttonBox.button(7).and(intakeTrigger).onTrue(new Feed());
    // buttonBox.button(7).and(intakeTrigger).onTrue(new ParallelCommandGroup(new InstantCommand(()
    // -> s_Intake.runIntake()), new SequentialCommandGroup(new FeedUntillSensor(), new
    // RepositionNote())));
    //    buttonBox.button(7).and(indexTrigger).onTrue(new ParallelCommandGroup(
    //                 new InstantCommand(() -> s_Shooter.stopShooter()),
    //                 new InstantCommand(() -> s_Index.indexStop(), s_Index)).andThen(new
    // TravelMode()));
    buttonBox
        .button(7)
        .and(climberTrigger)
        .onTrue(
            new InstantCommand(
                () ->
                    s_Climber.climberTo(
                        Constants.ClimberConstants.leftClimbedPosStable,
                        Constants.ClimberConstants.rightClimbedPosStable)));
    buttonBox
        .button(7)
        .and(climberTrigger.negate())
        .onTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> s_Shooter.shooterTo(50)),
                new InstantCommand(() -> s_Shooter.babyBird(200, 200)),
                new InstantCommand(() -> s_Index.babyBirdIndex())));

    buttonBox
        .button(7)
        .onFalse(
            new ParallelCommandGroup(
                new InstantCommand(() -> s_Shooter.stopShooter()),
                new InstantCommand(() -> s_Index.indexStop(), s_Index)));

    buttonBox.button(8).onTrue(new InstantCommand(() -> s_Intake.vomit()));
    buttonBox.button(8).onTrue(new InstantCommand(() -> s_Index.setIndexPower(-.3)));
    buttonBox.button(8).onFalse(new InstantCommand(() -> s_Intake.stopIntake()));
    buttonBox.button(8).onFalse(new InstantCommand(() -> s_Index.indexStop()));
   
    buttonBox.button(9).onTrue(new SpeakerMode().alongWith(new SpeakerLEDMode(m_LED)));
    buttonBox.button(9).onTrue(new InstantCommand(() -> s_Shooter.shooterTo(Constants.ShooterConstants.shooterSpeaker)));
    // buttonBox.button(9).and(ampTrigger).onTrue(new InstantCommand(() ->
    // s_Shooter.setPower(.11)));
    buttonBox
        .button(9)
        .and(climberTrigger)
        .onTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> s_Shooter.stopShooter(), s_Shooter),
                new InstantCommand(() -> s_Index.indexStop())));

    buttonBox
        .button(10)
        .and(climberTrigger)
        .onTrue(new InstantCommand(() -> s_Climber.raiseCimber(), s_Climber));
    // buttonBox.button(10).and(ampTrigger).onTrue(shimmy);
    buttonBox.button(11).and(ampTrigger).onTrue(new InstantCommand(() -> s_Flipper.setFlipperUp()));
    buttonBox
        .button(11)
        .and(ampTrigger)
        .onFalse(new InstantCommand(() -> s_Flipper.setFlipperDown()));

    buttonBox
        .button(11)
        .and(climberTrigger)
        .onTrue(new InstantCommand(() -> s_Shooter.stopAngle(), s_Shooter));
    buttonBox
        .button(11)
        .and(climberTrigger)
        .onTrue(
            // new InstantCommand(() -> s_Climber.lowerClimber())
            new MagicClimb(s_Climber, drivetrain));
    buttonBox
        .button(11)
        .and(speakerTrigger)
        .onTrue(new InstantCommand(() -> s_Shooter.shooterTo()));
    buttonBox
        .button(12)
        .and(ampTrigger.negate())
        .and(travelTrigger.negate())
        .onTrue(new InstantCommand(() -> s_Index.setIndexRPM(s_StateController.getIndexSpeed())));

    buttonBox
        .button(12)
        .and(travelTrigger)
        .onTrue(
            new InstantCommand(
                    () ->
                        s_Shooter.setShooterRPM(
                            Constants.ShooterConstants.leftShooterSpeakerRPM,
                            Constants.ShooterConstants.rightShooterSpeakerRPM))
                .andThen(
                    Commands.waitSeconds(.3)
                        .andThen(
                            new InstantCommand(
                                () ->
                                    s_Index.setIndexRPM(
                                        Constants.IndexConstants.indexSpeakerRPM)))));

    buttonBox.button(12).and(ampTrigger).onTrue(new AmpFireNew());

    buttonBox
        .button(12)
        .and(climberTrigger)
        .onTrue(new InstantCommand(() -> s_Climber.stopClimber()));
    buttonBox
        .button(12)
        .and(climberTrigger)
        .onTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> s_Shooter.stopShooter(), s_Shooter),
                new InstantCommand(() -> s_Index.indexStop())));
    buttonBox
        .button(12)
        .and(ampTrigger.negate())
        .onFalse(
            new ParallelCommandGroup(
                new InstantCommand(() -> s_Shooter.stopShooterRPM()),
                new InstantCommand(() -> s_Index.indexStop()),
                new TravelMode(m_LED)));
    buttonBox
        .button(12)
        .and(ampTrigger)
        .onFalse(
            new ParallelCommandGroup(
                    new InstantCommand(() -> s_Shooter.stopShooter(), s_Shooter),
                    new InstantCommand(() -> s_Index.indexStop(), s_Index),
                    new InstantCommand(() -> s_Flipper.setFlipperDown()))
                .andThen(new TravelMode(m_LED)));

    // driver.start().onTrue(new Aim(drivetrain));
    // driver.leftTrigger().and(travelTrigger).onTrue(aimAndShootCommand.andThen(new
    // InstantCommand(() ->
    // s_Shooter.setShooterRPM(Constants.ShooterConstants.leftShooterSpeakerRPM,
    // Constants.ShooterConstants.rightShooterSpeakerRPM))).andThen(Commands.waitSeconds(.6)).andThen(new InstantCommand(()->s_Index.setIndexRPM(s_StateController.getIndexSpeed()))));
    driver
        .a()
        .and(speakerTrigger)
        .onTrue(
            aimAndShootCommandAmp
                .andThen(Commands.waitSeconds(.4))
                .andThen(
                    new InstantCommand(
                        () -> s_Index.setIndexRPM(s_StateController.getIndexSpeed()))));
    driver
        .a()
        .onFalse(
            new ParallelCommandGroup(
                new InstantCommand(() -> s_Shooter.stopShooterRPM()),
                new InstantCommand(() -> s_Index.indexStop()),
                new TravelMode(m_LED)));
    driver
        .leftTrigger()
        .and(speakerTrigger)
        .onTrue(
            aimAndShootCommand
                .andThen(Commands.waitSeconds(.4))
                .andThen(
                    new InstantCommand(
                        () -> s_Index.setIndexRPM(s_StateController.getIndexSpeed()))));
    driver
        .leftTrigger()
        .onFalse(
            new ParallelCommandGroup(
                new InstantCommand(() -> s_Shooter.stopShooterRPM()),
                new InstantCommand(() -> s_Index.indexStop()),
                new TravelMode(m_LED)));
    // driver.a().onTrue(aimAndShootCommandAmp);
    driver.y().onTrue(new RAINBOW(m_LED));
    driver.y().and(driver.b()).onTrue(new InstantCommand(() -> s_Flipper.panic()));
    driver.y().onFalse(new InstantCommand(() -> s_Flipper.setFlipperDown()));

    // driver.start().onTrue(new AutoShootAngle(drivetrain, s_Shooter,s_StateController));
    // driver.start()
    //             .onTrue(drivetrain.addMeasurementCommand(() -> getBestPose()));
    pit.b().onTrue(new InstantCommand(() -> s_Climber.climberDown()));
    pit.b().onFalse(new InstantCommand(() -> s_Climber.stopClimber()));
    pit.a().onTrue(new InstantCommand(() -> s_Climber.climberUp()));
    pit.a().onFalse(new InstantCommand(() -> s_Climber.stopClimber()));

    pit.x().and(pit.pov(0)).whileTrue(drivetrain.runDriveQuasiTest(Direction.kForward));
    pit.x().and(pit.pov(180)).whileTrue(drivetrain.runDriveQuasiTest(Direction.kReverse));

    pit.y().and(pit.pov(0)).whileTrue(drivetrain.runDriveDynamTest(Direction.kForward));
    pit.y().and(pit.pov(180)).whileTrue(drivetrain.runDriveDynamTest(Direction.kReverse));

    pit.rightBumper().onTrue(new InstantCommand(() -> s_Intake.cleam()));
    pit.rightBumper().onFalse(new InstantCommand(() -> s_Intake.stopIntake()));

    // driver.start().onFalse(new InstantCommand(() -> s_Flipper.stopFlipper()));
    // driver.y().and(indexTrigger.negate()).onTrue(
    //     new ParallelCommandGroup(
    //         new InstantCommand(()-> s_Shooter.setShooterRPM(200, -200)),
    //         new InstantCommand(()-> s_Shooter.setIndexPower(.2)))
    // );
    // driver.povDown().onFalse(new InstantCommand(() -> s_Flipper.setFlipperDown()));
    // driver
    //     .y()
    //     .and(indexTrigger.negate())
    //     .onFalse(
    //         new ParallelCommandGroup(
    //             new InstantCommand(() -> s_Shooter.stopShooter()),
    //             new InstantCommand(() -> s_Index.indexStop())));
    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public void clear() {
    s_Intake.stopIntake();
    s_Shooter.stopShooter();
    s_Index.indexStop();
    s_Intake.setHingeTo(Constants.IntakeConstants.hingeUp);
    s_Flipper.setFlipperDown();
    m_LED.setColor(255, 255, 255);
    // m_LED.rainbow();
    // s_rAMP.setrAMPUp();
    
  }

  public RobotContainer() {
    NamedCommands.registerCommand(
        "rotationOverride", new InstantCommand(() -> drivetrain.setAutoAimPath(true)));
    NamedCommands.registerCommand(
        "NOverride", new InstantCommand(() -> drivetrain.setAutoAimPath(false)));
    NamedCommands.registerCommand(
        "rampDown", new InstantCommand(() -> s_Flipper.setFlipperDown(), s_Flipper));
    NamedCommands.registerCommand(
        "runIntakeFaster",
        new SequentialCommandGroup(
            new InstantCommand(() -> s_Intake.runIntake(), s_Intake),
            new FeedUntillSensor(m_LED, s_Intake),
            new RepositionNoteAuto(s_Index, s_Intake),
            new LookUpTableInst(s_Shooter, drivetrain)));

    NamedCommands.registerCommand(
        "runIntake",
        new SequentialCommandGroup(
            new InstantCommand(() -> s_Intake.runIntake(), s_Intake),
            new FeedUntillSensor(m_LED, s_Intake),
            new RepositionNoteAuto(s_Index, s_Intake)));
    NamedCommands.registerCommand(
        "stopShooter", 
        new InstantCommand(() -> s_Shooter.stopShooterRPM(), s_Shooter)
        );
    NamedCommands.registerCommand(
        "intakeDown",
        new InstantCommand(
            () -> s_Intake.setHingeTo(Constants.IntakeConstants.hingeDown), s_Intake));
    NamedCommands.registerCommand(
        "intakeUp",
        new InstantCommand(() -> s_Intake.setHingeTo(Constants.IntakeConstants.hingeUp)));

    NamedCommands.registerCommand(
        "revShooter", new InstantCommand(() -> s_Shooter.setShooterRPM(-6000, 6000), s_Shooter));
    NamedCommands.registerCommand(
        "revShooterFaster",
        new InstantCommand(() -> s_Shooter.setShooterRPM(-6000, 6000), s_Shooter));
    //    NamedCommands.registerCommand( "revShooter", Commands.print("marker1"));
    NamedCommands.registerCommand("fire", new InstantCommand(() -> s_Index.setIndexRPM(-6000)));
    NamedCommands.registerCommand("stopIntake", new InstantCommand(() -> s_Intake.stopIntake()));
    NamedCommands.registerCommand("runIndex", new InstantCommand(() -> s_Index.setIndexPower(-.4)));
    NamedCommands.registerCommand(
        "stopIndex", new InstantCommand(() -> s_Index.indexStop(), s_Shooter));
    NamedCommands.registerCommand(
        "shooterHome", new InstantCommand(() -> s_Shooter.shooterTo(65), s_Shooter));

    NamedCommands.registerCommand(
        "shooterSpeaker", new InstantCommand(() -> s_Shooter.shooterTo(56), s_Shooter));

    NamedCommands.registerCommand(
        "shooterTravel", new InstantCommand(() -> s_Shooter.shooterTo(12), s_Shooter));
    NamedCommands.registerCommand("varAngle", new LookUpTableAuton(s_Shooter, drivetrain));
    NamedCommands.registerCommand("autoAim", new RotateToSpeaker(drivetrain));

    m_LED.setColor(255, 255, 255);
    // rightLED.setColor(255, 255, 255);
    
    aimAndShootCommandAmp =
        Commands.runOnce(
                () -> {
                  var pose = getBestPose();
                  if (pose.isPresent())
                    drivetrain.seedFieldRelative(pose.get().estimatedPose.toPose2d());
                })
            .andThen(Commands.waitSeconds(.1))
            .andThen(new RotateToAmp(drivetrain));
    aimAndShootCommand =
        Commands.runOnce(
                () -> {
                  var pose = getBestPose();
                  if (pose.isPresent())
                    drivetrain.seedFieldRelative(pose.get().estimatedPose.toPose2d());
                })
            .andThen(Commands.waitSeconds(.1))
            .andThen(new RotateToSpeaker(drivetrain));
    configureBindings();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  public Optional<EstimatedRobotPose> getBestPose() {
    Pose2d drivetrainPose = drivetrain.getState().Pose;

    Optional<EstimatedRobotPose> front = rightVision.getCameraResult(drivetrainPose);
    Optional<EstimatedRobotPose> back = leftVision.getCameraResult(drivetrainPose);

    int numPoses = 0;

    numPoses += front.isPresent() ? 1 : 0;
    numPoses += back.isPresent() ? 1 : 0;
    Optional<Pose2d> pose = Optional.empty();
    SmartDashboard.putNumber("numPoses", numPoses);

    if (numPoses == 1) {
      pose =
          Optional.of(
              new Pose2d(
                  (front.isEmpty() ? back : front).get().estimatedPose.toPose2d().getTranslation(),
                  drivetrainPose.getRotation()));
    } else if (numPoses == 2) {
      // average the poses
      Pose3d frontP = front.get().estimatedPose;
      Pose3d backP = back.get().estimatedPose;

      Translation2d frontT = frontP.getTranslation().toTranslation2d();
      Translation2d backT = backP.getTranslation().toTranslation2d();

      pose = Optional.of(new Pose2d(frontT.plus(backT).div(2.), drivetrainPose.getRotation()));
    }

    if (pose.isPresent()) {
      return Optional.of(
          new EstimatedRobotPose(
              new Pose3d(pose.get()),
              (front.isEmpty() ? back : front).get().timestampSeconds,
              null,
              null));
    }

    return Optional.empty();
  }

  public boolean distanceFilter() {
    return leftVision.getDistOfTag(drivetrain.getState().Pose) < 5
        || rightVision.getDistOfTag(drivetrain.getState().Pose) < 5;
  }

  public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
    if (leftVision.getHasTarget()) return leftVision.getEstimationStdDevs(estimatedPose);
    if (rightVision.getHasTarget()) return rightVision.getEstimationStdDevs(estimatedPose);
    return Constants.VisionConstants.kSingleTagStdDevs;
  }

  //   public double getAngle() {
  //       if(leftVision.getTargetById(4)!=null) distance = leftVision.getTagDistance(4).get();
  //       if(rightVision.getTargetById(4)!=null) distance = leftVision.getTagDistance(4).get();
  //       SmartDashboard.putNumber("angleto", Math.atan(distance/60));
  //       return (Math.atan(distance/60));

  //   }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
