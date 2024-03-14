// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.vision;

// import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
// import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.AddressableLED;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.PIDCommand;
// import frc.robot.generated.TunerConstants;
// import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.Vision;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class AimPID extends PIDCommand {
//   /** Creates a new AimPID. */
//   private double MaxSpeed =
//       TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
//   private double MaxAngularRate =
//       1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
//   CommandSwerveDrivetrain m_Swerve;
//   private double direction;
//   private double speed;
//   // private  SwerveRequest.FieldCentric drive =
//   //     new SwerveRequest.FieldCentric()
//   //         .withDeadband(MaxSpeed*.1)
//   //         .withRotationalDeadband(MaxAngularRate*.1) 
//   //         .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

//   Vision m_Vision; 
//   public AimPID(Vision vision, CommandSwerveDrivetrain swerve, SwerveRequest.FieldCentric drive) {
//     super(
//         // The controller that the command will use
//         new PIDController(1, 0, 0.2),
//         // This should return the measurement
//         () -> vision.getTxSpeaker(),
//         // This should return the setpoint (can also be a constant)
//         () -> 0,
//         // This uses the output
//         output -> {
//           // Use the output here
//           SmartDashboard.putNumber("pidout", output);
//           SmartDashboard.putNumber("txSpeaker", vision.getTxSpeaker());
//           swerve.setControl(drive.withVelocityX(0).withVelocityY(0).withRotationalRate(-output));

//         }, swerve);
//         getController().setTolerance(.4, 1);
//         getController().enableContinuousInput(-180, 180);
        
//     // Use addRequirements() here to declare subsystem dependencies.
//     m_Swerve = swerve; 
//     m_Vision = vision;
//     addRequirements(m_Vision, m_Swerve);
//     // Configure additional PID options by calling `getController` here.
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
