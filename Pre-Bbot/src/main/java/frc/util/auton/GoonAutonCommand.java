// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.util.auton;

import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Add your docs here. */
public class GoonAutonCommand extends SequentialCommandGroup {

    private Pose2d initialPose;

    public GoonAutonCommand() {
        // Add your commands in the addCommands() call, e.g.
        
      }
    
    protected void setInitialPose(PathPlannerTrajectory initialTrajectory) {
        this.initialPose = new Pose2d(initialTrajectory.getInitialDifferentialPose().getTranslation(),
                initialTrajectory.getInitialState().targetHolonomicRotation);
    }
    
    public Pose2d getInitialPose() {
        return initialPose;
    }

}
