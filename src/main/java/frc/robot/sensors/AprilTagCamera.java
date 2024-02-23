// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

//import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
//import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
//import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

//import edu.wpi.first.apriltag.AprilTagFieldLayout;
//import edu.wpi.first.apriltag.AprilTagFields;
//import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Pose3d;
//import edu.wpi.first.math.geometry.Rotation3d;
//import edu.wpi.first.math.geometry.Transform3d;
//import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
//import edu.wpi.first.wpilibj.DriverStation.Alliance;
//import frc.robot.Constants.APRILTAGS;
import frc.robot.interfaces.ICamera;

/** Wrapper for PhotonCamera class */
public class AprilTagCamera extends PhotonCamera implements ICamera {

    //TODO: UPDATE CAM SETTINGS FOR NEW ROBOT
    private static final String DEFAULT_CAM_NAME = "AprilTagCamera";
    private final double CAMERA_HEIGHT_METERS =  Units.inchesToMeters(52.25); // 52in up from center
    private final double TARGET_HEIGHT_METERS = 0.36; // may need to change 
    private final int CAMERA_PITCH_RADIANS = 0; // tilt of our camera (radians)

    public AprilTagCamera() {
        super(DEFAULT_CAM_NAME);
    }

    public double getDistanceToTarget() {
        PhotonPipelineResult result = getLatestResult();

        if (result.hasTargets()) {
            double range = PhotonUtils.calculateDistanceToTargetMeters(
                CAMERA_HEIGHT_METERS, TARGET_HEIGHT_METERS, CAMERA_PITCH_RADIANS, 
                Units.degreesToRadians(getPitch())
            );
            return Units.metersToInches(range);
        }
        return 0.0;
    }

    public double getYaw() {
        /* The yaw of the target in degrees (positive right). */
        return getLatestResult().hasTargets() ? 
            getLatestResult().getBestTarget().getYaw():
            0.0;
    }

    public double getAngleToTurnToTarget()
    {
        return +getYaw();
    }

    public double getPitch() {
        /* The pitch of the target in degrees (positive up). */
        return getLatestResult().hasTargets() ? 
            getLatestResult().getBestTarget().getPitch():
            0.0;
    }

    public double getSkew() {
        /* The skew of the target in degrees (counter-clockwise positive). */
        return getLatestResult().hasTargets() ? 
            getLatestResult().getBestTarget().getSkew():
            0.0;
    }

    public int getApriltagID() {
        return getLatestResult().hasTargets() ? 
            getLatestResult().getBestTarget().getFiducialId():
            0;
    }
}

