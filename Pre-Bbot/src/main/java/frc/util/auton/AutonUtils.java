// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.util.auton;

import java.util.HashMap;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

/** Add your docs here. */
public class AutonUtils {
    
    //public FollowPathHolonomic followTrajectoryCommand(PathPlannerTrajectory traj) {
        
        // return new FollowPathHolonomic(
        //     traj,
        //     SwerveDrive.getInstance()::getPose,
        //     SwerveDrive.getInstance()::getSpeeds,
        //     SwerveDrive.getInstance()::driveRobotRelative,
        //     Constants.Swerve.pathFollowerConfig,
        //     true,
        //     SwerveDrive.getInstance());
                        


        
       
   // }

// public static FollowPathWithEvents getPathWithEvents(PathplannerTrajectory traj, HashMap<String, Command> eventMap){
//     return new FollowPathWithEvents(null, null, null)
// }

}
