// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private static Intake _instance;
  private WPI_TalonSRX spikeWheel;
  private WPI_TalonSRX hingeMotor;
  private WPI_TalonSRX solidWheel;
  
  private ElevatorFeedforward hingeFeedForward;
  private DutyCycleEncoder hingeEncoder;
  private TrapezoidProfile.Constraints m_constraints;
  private ProfiledPIDController hingePIDController;

  private double lastSpeed;
  private double lastTime;


  
  public Intake() {

    spikeWheel = new WPI_TalonSRX(Constants.IntakeConstants.spikeWheelID);
    hingeMotor = new WPI_TalonSRX(Constants.IntakeConstants.hingeID);
    hingeMotor.setNeutralMode(NeutralMode.Brake);
    solidWheel = new WPI_TalonSRX(Constants.IntakeConstants.solidWheelID);

    hingeEncoder = new DutyCycleEncoder(0);
    hingeEncoder.reset();

    m_constraints = new TrapezoidProfile.Constraints(2.75, 0.75);

    hingePIDController = new ProfiledPIDController(100, 0, 0, m_constraints, 0.02);

    hingeFeedForward = new ElevatorFeedforward(1.1, 1.2, 1.3);

  }

   /** Creates a new Intake. */
  public static Intake getInstance(){
    if (_instance == null){
      _instance = new Intake();
    }
    return _instance;
  }

  public void setIntakeSpeed(double speed){
    spikeWheel.set(speed);
    solidWheel.set(-speed);
  }

  public void setHingeTo(double position){
    hingePIDController.setGoal(position);
    double pidVal = hingePIDController.calculate(hingeEncoder.getDistance(), position);
    double acceleration = (hingePIDController.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);
    hingeMotor.setVoltage(
      pidVal
      + hingeFeedForward.calculate(hingePIDController.getSetpoint().velocity, acceleration));

    lastSpeed = hingePIDController.getSetpoint().velocity;
    lastTime = Timer.getFPGATimestamp();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("encoder pos", hingeEncoder.getDistance());

    hingeMotor.setVoltage(
        hingePIDController.calculate(hingeEncoder.getDistance())
            + hingeFeedForward.calculate(hingePIDController.getSetpoint().velocity));
  }
}
