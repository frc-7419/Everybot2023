// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems.photonvision;

import java.util.ArrayList;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.FieldConstants;
import java.util.ArrayList;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import frc.robot.Constants.VisionConstants;
import org.photonlib.PhotonPoseEstimator;
import org.photonlib.PhotonPoseEstimator.PoseStrategy;

/** Add your docs here. */

/*
idk wtf this is, it said it was neccesary in the documentation so...

also why is everything broken 
i tried to import the photonposeestimator but it didnt work :(
comment out for now
*/


public class PhotonCamWrapper {
    // field
    public PhotonCamera photonCamera;
    public PhotonPoseEstimator photonPoseEstimator;
    
    public void PhotonCameraWrapper() {
        final AprilTag tag18 =
                        new AprilTag(
                                18,
                                new Pose3d(
                                        new Pose2d(
                                                FieldConstants.length,
                                                FieldConstants.width / 2.0,
                                                Rotation2d.fromDegrees(180))));

        
        final AprilTag tag01 = 
        new AprilTag(01, new Pose3d(new Pose2d(0.0, FieldConstants.width / 2.0, Rotation2d.fromDegrees(0.0))));

        ArrayList<AprilTag> atList = new ArrayList<AprilTag>();
        
       
        atList.add(tag18);
        atList.add(tag01);

        AprilTagFieldLayout atfl = new AprilTagFieldLayout(atList, FieldConstants.length, FieldConstants.width);

        photonCamera =
        new PhotonCamera(VisionConstants.boboticsCam);


        photonPoseEstimator = 
        new PhotonPoseEstimator(atfl, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, photonCamera, VisionConstants.robotToCam);

    }
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }
}

