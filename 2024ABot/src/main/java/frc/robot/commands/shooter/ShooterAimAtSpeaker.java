// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.shooter;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.Shooter;
// import frc.robot.subsystems.StateController;
// import frc.util.RobotMode;

// public class ShooterAimAtSpeaker extends Command {
//   /** Creates a new ShooterAimAtSpeaker. */

//   Shooter m_shooter;
//   CommandSwerveDrivetrain m_drive;
//   StateController m_StateController;
//   double angle;
//   double distance;
//   Pose2d target;
//   public ShooterAimAtSpeaker(CommandSwerveDrivetrain drive, Shooter shoot, StateController
// statecontroller) {
//     m_shooter = shoot;
//     m_drive = drive;
//     m_StateController = statecontroller;
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void init() {
//      Pose2d pose = m_drive.getState().Pose;
//         target =Constants.VisionConstants.RED_SPEAKER_DISTANCE_TARGET;

//         var alliance = DriverStation.getAlliance();
//           if (alliance.isPresent()) {
//              if(alliance.get() == DriverStation.Alliance.Blue)target =
// Constants.VisionConstants.BLUE_SPEAKER_DISTANCE_TARGET;
//           }
//          distance = pose.getTranslation().getDistance(target.getTranslation());
//         angle = Math.atan(1.524/(distance-0.2286));
//           if(m_StateController.getMode() == RobotMode.SPEAKER && Math.toDegrees(angle)<56 &&
// Math.toDegrees(angle)>13) {
//             m_shooter.shooterTo(Math.toDegrees(angle));
//       }
// }
//   // Called every time the scheduler runs while the command is scheduled.

//   // Returns true when the command should end.

// }
