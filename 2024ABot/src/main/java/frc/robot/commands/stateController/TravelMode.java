// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.stateController;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Flipper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.StateController;
import frc.robot.subsystems.rAMP;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TravelMode extends InstantCommand {
  Intake m_Intake;
  Shooter m_Shooter;
  StateController m_StateController;
  Flipper m_flipper;
  LED leftLED;
  LED rightLED;

  public TravelMode(LED left) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Intake = Intake.getInstance();
    m_Shooter = Shooter.getInstance();
    m_StateController = StateController.getInstance();
    m_flipper = Flipper.getInstance();
    leftLED = left;
    addRequirements(m_Intake, m_Shooter, m_StateController, m_flipper, leftLED);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Intake.setHingeTo(Constants.IntakeConstants.hingeUp);
    m_StateController.setTravel();
    m_Shooter.shooterTo(m_StateController.getAngle());
    m_Shooter.setPower(0);
    m_flipper.setFlipperDown();
    leftLED.setColor(200, 0, 255);
  }
}
