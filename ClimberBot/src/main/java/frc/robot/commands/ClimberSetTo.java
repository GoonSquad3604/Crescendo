// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.PIDCommand;
// import frc.robot.subsystems.Climber;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// //public class ClimberSetTo extends PIDCommand {

//   //public boolean finished = false;

//   //Climber s_Climber;
//   /** Creates a new ClimberSetTo. */
//   //public ClimberSetTo(Climber climber) {
//     // super(
//     //     // The controller that the command will use
//     //     new PIDController(8.55, 0, 0),
//     //     // This should return the measurement
//     //     () -> climber.getAbsolutePosition(),
//     //     // This should return the setpoint (can also be a constant)
//     //     () -> climber.getSetPoint(),
//     //     // This uses the output
//     //     output -> {
//     //       climber.setPower(output);
//     //     });

//         addRequirements(climber);
//     // Use addRequirements() here to declare subsystem dependencies.
//     // Configure additional PID options by calling `getController` here.
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return getController().atSetpoint();
//   }
// }
