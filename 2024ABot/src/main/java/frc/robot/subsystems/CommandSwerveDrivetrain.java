package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.util.SwerveVoltageRequest;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;
  private final Field2d m_field = new Field2d();
  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean hasAppliedOperatorPerspective = false;

  @AutoLogOutput private boolean autoAimInPath = false;

  @AutoLogOutput private Pose2d robotPose;

  private final SwerveRequest.ApplyChassisSpeeds AutoRequest =
      new SwerveRequest.ApplyChassisSpeeds();

  private final SwerveRequest.SysIdSwerveTranslation TranslationCharacterization =
      new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveRotation RotationCharacterization =
      new SwerveRequest.SysIdSwerveRotation();
  private final SwerveRequest.SysIdSwerveSteerGains SteerCharacterization =
      new SwerveRequest.SysIdSwerveSteerGains();

  /* Use one of these sysidroutines for your particular test */
  private SysIdRoutine SysIdRoutineTranslation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Volts.of(4),
              null,
              (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
          new SysIdRoutine.Mechanism(
              (volts) -> setControl(TranslationCharacterization.withVolts(volts)), null, this));

  private final SysIdRoutine SysIdRoutineRotation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Volts.of(4),
              null,
              (state) -> Logger.recordOutput("Rotation/SysIdState", state.toString())),
          new SysIdRoutine.Mechanism(
              (volts) -> setControl(RotationCharacterization.withVolts(volts)), null, this));
  private final SysIdRoutine SysIdRoutineSteer =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Volts.of(7),
              null,
              (state) -> Logger.recordOutput("Steer/SysIdState", state.toString())),
          new SysIdRoutine.Mechanism(
              (volts) -> setControl(SteerCharacterization.withVolts(volts)),
              null,
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("drive")
              this));

  /* Change this to the sysid routine you want to test */
  private final SysIdRoutine RoutineToApply = SysIdRoutineTranslation;

  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants driveTrainConstants,
      double OdometryUpdateFrequency,
      SwerveModuleConstants... modules) {
    super(driveTrainConstants, OdometryUpdateFrequency, modules);

    configurePathPlanner();

    if (Utils.isSimulation()) {
      startSimThread();
    }
    SmartDashboard.putData("Field", m_field);
  }

  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
    super(driveTrainConstants, modules);

    configurePathPlanner();
    if (Utils.isSimulation()) {
      startSimThread();
    }
    SmartDashboard.putData("Field", m_field);
  }

  private void configurePathPlanner() {
    double driveBaseRadius = 0;
    for (var moduleLocation : m_moduleLocations) {
      driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
    }

    AutoBuilder.configureHolonomic(
        () -> this.getState().Pose, // Supplier of current robot pose
        this::seedFieldRelative, // Consumer for seeding pose against auto
        this::getCurrentRobotChassisSpeeds,
        (speeds) ->
            this.setControl(
                AutoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
        new HolonomicPathFollowerConfig(
            new PIDConstants(11, 0, 0),
            new PIDConstants(8, 0, 0),
            TunerConstants.kSpeedAt12VoltsMps,
            driveBaseRadius,
            new ReplanningConfig()),
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        }, // Change this if the path needs to be flipped on red vs blue
        this); // Subsystem for requirements
    PPHolonomicDriveController.setRotationTargetOverride(this::getRotationToSpeakerTarget);
  }

  public Optional<Rotation2d> getRotationToSpeakerTarget() {

    if (autoAimInPath) {
      Pose2d target = Constants.VisionConstants.RED_SPEAKER_DISTANCE_TARGET;
      Pose2d pose = this.getState().Pose;

      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
        if (alliance.get() == DriverStation.Alliance.Blue)
          target = Constants.VisionConstants.BLUE_SPEAKER_DISTANCE_TARGET;
      }
      Translation2d distance =
          new Translation2d(pose.getX() - target.getX(), pose.getY() - target.getY());
      Rotation2d angle = distance.getAngle();

      return Optional.of(angle);
    }
    return Optional.empty();
  }

  public void setAutoAimPath(boolean aimSpeaker) {
    autoAimInPath = aimSpeaker;
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  public Command getAutoPath(String pathName) {
    return new PathPlannerAuto(pathName);
  }

  public Command addMeasurementCommand(Supplier<Pose2d> measurement, Supplier<Double> timeStamp) {
    return runOnce(
        () -> {
          Pose2d pose = measurement.get();
          if (pose == new Pose2d()) return;

          addVisionMeasurement(pose, timeStamp.get());
        });
  }

  public Command addMeasurementCommand(Supplier<Optional<EstimatedRobotPose>> pose) {
    return runOnce(
        () -> {
          Optional<EstimatedRobotPose> estPose = pose.get();
          if (estPose.isPresent()) {
            addVisionMeasurement(
                estPose.get().estimatedPose.toPose2d(), estPose.get().timestampSeconds);
          }
        });
  }

  /*
   * Both the sysid commands are specific to one particular sysid routine, change
   * which one you're trying to characterize
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return RoutineToApply.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return RoutineToApply.dynamic(direction);
  }

  public ChassisSpeeds getCurrentRobotChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(getState().ModuleStates);
  }

  private SwerveVoltageRequest driveVoltageRequest = new SwerveVoltageRequest(true);
  // Logger.recordOutput("Drive/SysIdState", state.toString())
  private SysIdRoutine m_driveSysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, null, null, (state) -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              (Measure<Voltage> volts) ->
                  setControl(driveVoltageRequest.withVoltage(volts.in(Volts))),
              null,
              this));

  private SwerveVoltageRequest steerVoltageRequest = new SwerveVoltageRequest(false);

  private SysIdRoutine m_steerSysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, null, null, (state) -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              (Measure<Voltage> volts) ->
                  setControl(steerVoltageRequest.withVoltage(volts.in(Volts))),
              null,
              this));

  // private SysIdRoutine m_slipSysIdRoutine =
  // new SysIdRoutine(
  //     new SysIdRoutine.Config(Volts.of(0.25).per(Seconds.of(1)), null, null,
  // ModifiedSignalLogger.logState()),
  //     new SysIdRoutine.Mechanism(
  //         (Measure<Voltage> volts) ->
  // setControl(driveVoltageRequest.withVoltage(volts.in(Volts))),
  //         null,
  //         this));

  public Command runDriveQuasiTest(Direction direction) {
    return m_driveSysIdRoutine.quasistatic(direction);
  }

  public Command runDriveDynamTest(SysIdRoutine.Direction direction) {
    return m_driveSysIdRoutine.dynamic(direction);
  }

  public Command runSteerQuasiTest(Direction direction) {
    return m_steerSysIdRoutine.quasistatic(direction);
  }

  public Command runSteerDynamTest(SysIdRoutine.Direction direction) {
    return m_steerSysIdRoutine.dynamic(direction);
  }

  // public Command runDriveSlipTest()
  // {
  //     return m_slipSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
  // }
  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - m_lastSimTime;
              m_lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  @Override
  public void periodic() {
    m_field.setRobotPose(this.getState().Pose);

    robotPose = this.getState().Pose;
    // SmartDashboard.putNumber("xPose",this.getState().Pose.getX());
    // SmartDashboard.putNumber("yPose", this.getState().Pose.getY());
    /* Periodically try to apply the operator perspective */
    /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
    /* This allows us to correct the perspective in case the robot code restarts mid-match */
    /* Otherwise, only check and apply the operator perspective if the DS is disabled */
    /* This ensures driving behavior doesn't change until an explicit disable event occurs during testing*/
    if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              (allianceColor) -> {
                this.setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? RedAlliancePerspectiveRotation
                        : BlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
              });
    }
  }
}
