// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;


import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;


import edu.wpi.first.math.util.Units;
import frc.robot.interfaces.ICamera;

/** Wrapper for PhotonCamera class */
public class ObjectDetectionCamera extends PhotonCamera implements ICamera {

    //TODO: UPDATE CAM SETTINGS FOR NEW ROBOT
    private static final String DEFAULT_CAM_NAME = "ObjectDetectionCam";
    private final double CAMERA_HEIGHT_METERS = 0.356; // height on robot (meters)
    private final double TARGET_HEIGHT_METERS = 0.0; // may need to change 
    private final double CAMERA_PITCH_RADIANS = -10.0 * (Math.PI / 180.0); // tilt of our camera (radians)

    public ObjectDetectionCamera() {
        super(DEFAULT_CAM_NAME);
    }

    public double getYaw() {
        /* The yaw of the target in degrees (positive right). */
        return getLatestResult().hasTargets() ? 
            getLatestResult().getBestTarget().getYaw() :
            0.0;
    }

    public double getAngleToTurnToTarget()
    {
        return +getYaw();
    }

    public double getPitch() {
        /* The pitch of the target in degrees (positive up). */
        return getLatestResult().getBestTarget().getPitch();
    }

    public double getSkew() {
        /* The skew of the target in degrees (counter-clockwise positive). */
        return getLatestResult().getBestTarget().getSkew();
    }

    public double getDistanceToTarget() {
        PhotonPipelineResult result = getLatestResult();
        if (result.hasTargets()) {
            double range = PhotonUtils.calculateDistanceToTargetMeters(
                CAMERA_HEIGHT_METERS, TARGET_HEIGHT_METERS, CAMERA_PITCH_RADIANS, 
                Units.degreesToRadians(result.getBestTarget().getPitch())
            );
            return range;
        }
        return 0.0;
    }
    
    // public Transform3d transformToNote() {
    //     Transform3d pose = getLatestResult().getBestTarget().getBestCameraToTarget();
    //     return pose;
    // }
}