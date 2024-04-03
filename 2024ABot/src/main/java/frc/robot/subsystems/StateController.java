// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.util.RobotMode;
import org.littletonrobotics.junction.AutoLogOutput;

public class StateController extends SubsystemBase {
  /** Creates a new StateController. */
  public static StateController _instance;

  // private Vision m_Vision;
  @AutoLogOutput private RobotMode m_Mode;
  private double leftShooterSpeed;
  private double rightShooterSpeed;

  private double shooterAngle;
  private double indexSpeed;

  private boolean intakeEnabled;

  public StateController() {
    // m_Vision = Vision.getInstance();
    setTravel();
  }

  public static StateController getInstance() {

    if (_instance == null) {
      _instance = new StateController();
    }
    return _instance;
  }

  public void setIntake() {
    m_Mode = RobotMode.INTAKE;
    shooterAngle = Constants.ShooterConstants.shooterHome;
    leftShooterSpeed = 0;
    rightShooterSpeed = 0;
    intakeEnabled = true;
  }

  public void setTravel() {
    m_Mode = RobotMode.TRAVEL;
    shooterAngle = Constants.ShooterConstants.shooterTravel;
    intakeEnabled = false;
  }

  public void setClimber() {
    m_Mode = RobotMode.CLIMBER;
    intakeEnabled = false;
  }

  public void setTrap() {
    m_Mode = RobotMode.TRAP;
    indexSpeed = Constants.IndexConstants.indexTrapRPM;
    shooterAngle = Constants.ShooterConstants.shooterTrap;
    leftShooterSpeed = Constants.ShooterConstants.leftShooterTrapRPM;
    rightShooterSpeed = Constants.ShooterConstants.rightShooterTrapRPM;
  }

  public void setAmp() {
    m_Mode = RobotMode.AMP;
    indexSpeed = Constants.IndexConstants.indexAmpRPM;
    shooterAngle = Constants.ShooterConstants.shooterAmp;
    leftShooterSpeed = Constants.ShooterConstants.leftShooterAmpRPM;
    rightShooterSpeed = Constants.ShooterConstants.rightShooterAmpRPM;
  }

  public void setSpeaker() {
    m_Mode = RobotMode.SPEAKER;
    indexSpeed = Constants.IndexConstants.indexSpeakerRPM;
    shooterAngle = Constants.ShooterConstants.shooterSpeaker;
    leftShooterSpeed = Constants.ShooterConstants.leftShooterSpeakerRPM;
    rightShooterSpeed = Constants.ShooterConstants.rightShooterSpeakerRPM;

    // indexSpeed = m_Vision.getShooterSpeed() * 1.35;
    // leftShooterSpeed = m_Vision.getShooterSpeed();
    // rightShooterSpeed = m_Vision.getShooterSpeed();
    // shooterAngle = m_Vision.getShooterAngle();
  }

  public RobotMode getMode() {
    return m_Mode;
  }

  public double getIndexSpeed() {
    return indexSpeed;
  }

  public double getAngle() {
    return shooterAngle;
  }

  public double getLeftShooterSpeed() {
    return leftShooterSpeed;
  }

  public double getRightShooterSpeed() {
    return rightShooterSpeed;
  }

  public boolean isTravelMode() {
    return m_Mode == RobotMode.TRAVEL;
  }

  public boolean isIntakeMode() {
    return m_Mode == RobotMode.INTAKE;
  }

  public boolean isClimberMode() {
    return m_Mode == RobotMode.CLIMBER;
  }

  public boolean isAmpMode() {
    return m_Mode == RobotMode.AMP;
  }

  public boolean isSpeakerMode() {
    return m_Mode == RobotMode.SPEAKER;
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("state", m_Mode.toString());

    // This method will be called once per scheduler run
  }
}
