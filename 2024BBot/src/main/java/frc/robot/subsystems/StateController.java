// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.RobotMode;

public class StateController extends SubsystemBase {
  /** Creates a new StateController. */
  public static StateController _instance;
  private RobotMode m_Mode;
  public StateController() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
