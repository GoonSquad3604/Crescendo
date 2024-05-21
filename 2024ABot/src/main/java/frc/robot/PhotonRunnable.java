// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.RawSubscriber;

/** Add your docs here. */
public class PhotonRunnable implements Runnable{
    private final PhotonPoseEstimator[] photonPoseEstimators;
    private final RawSubscriber[] rawBytesSubscriber;
    private final int[] waitHandles;
    private final BiConsumer<Pose2d, Double> poseConsumer


    public PhotonRunnable(String[] camNames, Transform3d[] robotToCams, BiConsumer<Pose2d, Double> poseConsumer, Supplier<Pose2d> poseSupplier) {
        this.poseConsumer = poseConsumer;

    }


}
