package frc.robot.subsystems;

import frc.util.swerve.GoonSwerveModule;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDrive extends SubsystemBase {
  public static SwerveDrive _instance;
  public SwerveDriveOdometry swerveOdometry;
  public GoonSwerveModule[] mSwerveMods;
  public Pigeon2 gyro;

  public SwerveDrive() {
      gyro = new Pigeon2(Constants.Swerve.pigeonID, Constants.General.CANIVORE_CANBUS);
      gyro.getConfigurator().apply(new Pigeon2Configuration());
      gyro.setYaw(0);

      mSwerveMods = new GoonSwerveModule[] {
          new GoonSwerveModule(0, Constants.Swerve.Mod0.constants),
          new GoonSwerveModule(1, Constants.Swerve.Mod1.constants),
          new GoonSwerveModule(2, Constants.Swerve.Mod2.constants),
          new GoonSwerveModule(3, Constants.Swerve.Mod3.constants)
      };

      
      swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());

      AutoBuilder.configureHolonomic(
        this::getPose,
        this::setPose,
        this::getSpeeds,
        this::driveRobotRelative,
        Constants.Swerve.pathFollowerConfig,
        () -> {

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;

        },
        this
      );
        
  }

  public static final SwerveDrive getInstance() {
    if (_instance == null) {
            _instance = new SwerveDrive();
    }
    return _instance;
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
      SwerveModuleState[] swerveModuleStates =
          Constants.Swerve.swerveKinematics.toSwerveModuleStates(
              fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                  translation.getX(), 
                                  translation.getY(), 
                                  rotation, 
                                  getHeading()
                              )
                              : new ChassisSpeeds(
                                  translation.getX(), 
                                  translation.getY(), 
                                  rotation)
                              );
      SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

      for(GoonSwerveModule mod : mSwerveMods){
          mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
      }
  }    
  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds){
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(targetSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, Constants.Swerve.maxSpeed);
     for (GoonSwerveModule mod : mSwerveMods){
          mod.setDesiredState(targetStates[mod.moduleNumber], true);
      }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
      SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
      
      for(GoonSwerveModule mod : mSwerveMods){
          mod.setDesiredState(desiredStates[mod.moduleNumber], false);
      }
  }

  public SwerveModuleState[] getModuleStates(){
      SwerveModuleState[] states = new SwerveModuleState[4];
      for(GoonSwerveModule mod : mSwerveMods){
          states[mod.moduleNumber] = mod.getState();
      }
      return states;
  }

  public SwerveModulePosition[] getModulePositions(){
      SwerveModulePosition[] positions = new SwerveModulePosition[4];
      for(GoonSwerveModule mod : mSwerveMods){
          positions[mod.moduleNumber] = mod.getPosition();
      }
      return positions;
  }

  public Pose2d getPose() {
      return swerveOdometry.getPoseMeters();
  }

  public void setPose(Pose2d pose) {
      swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
  }

  public Rotation2d getHeading(){
      return getPose().getRotation();
  }

  public ChassisSpeeds getSpeeds() {
    return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
  }
  

  public void setHeading(Rotation2d heading){
      swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
  }

  public void zeroHeading(){
      swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
  }

  public Rotation2d getGyroYaw() {
      return Rotation2d.fromDegrees(gyro.getYaw().getValue());
  }

  public void resetModulesToAbsolute(){
      for(GoonSwerveModule mod : mSwerveMods){
          mod.resetToAbsolute();
      }
  }

  @Override
  public void periodic(){
      swerveOdometry.update(getGyroYaw(), getModulePositions());

      for(GoonSwerveModule mod : mSwerveMods){
          SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
          SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
          SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
      }
  }
}
