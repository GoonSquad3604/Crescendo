// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.vision;

// import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
// import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ForwardReference;
// import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
// import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
// import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.generated.TunerConstants;
// import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.Vision;

// public class BetterAim extends Command {
//   /** Creates a new BetterAim. */
//   private double MaxSpeed =
//       TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
//   private double MaxAngularRate =
//       1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
//   CommandSwerveDrivetrain m_Swerve;
//   Vision m_Vision;
//   private double direction;
//   private double speed;
//   private  SwerveRequest.FieldCentric drive =
//       new SwerveRequest.FieldCentric()
//           .withDeadband(MaxSpeed*.1)
//           .withRotationalDeadband(MaxAngularRate*.1) 
//           .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric

// private final SwerveRequest.FieldCentricFacingAngle swerveRequestFacing = new SwerveRequest.FieldCentricFacingAngle()
//     .withDriveRequestType(DriveRequestType.Velocity)
//     .withSteerRequestType(SteerRequestType.MotionMagic)
//     .withVelocityX(0.0)
//     .withVelocityY(0.0);
  
//   public BetterAim(CommandSwerveDrivetrain swerve, Vision m_Vision) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     m_Swerve = swerve;
//     this.m_Vision = m_Vision;
//     addRequirements(m_Swerve, this.m_Vision);
//     swerveRequestFacing.ForwardReference = ForwardReference.RedAlliance;
//     swerveRequestFacing.HeadingController = new PhoenixPIDController(.5, 0, 0);
//     swerveRequestFacing.HeadingController.enableContinuousInput(-3.14, 3.14);
//     swerveRequestFacing.HeadingController.setTolerance(.4);

//     swerveRequestFacing.ForwardReference = ForwardReference.RedAlliance;
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     swerveRequestFacing.HeadingController.reset();
    
//     // m_Swerve.getState().P
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//    var speakerTranslation = m_Vision.getSpeakerTranslation();
//     var driveTranslation = m_Swerve.getState().Pose.getTranslation();
//     var angleTo = speakerTranslation.minus(driveTranslation).getAngle();
//     m_Swerve.setControl(swerveRequestFacing.withVelocityX(0).withVelocityY(0).withTargetDirection(angleTo));
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
    
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
