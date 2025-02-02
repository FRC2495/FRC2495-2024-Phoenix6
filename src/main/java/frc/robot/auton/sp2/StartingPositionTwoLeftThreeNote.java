package frc.robot.auton.sp2;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.RobotContainer;
import frc.robot.auton.AutonConstants;
import frc.robot.auton.common.*;
import frc.robot.commands.drivetrain.*;
import frc.robot.commands.shooter.*;
import frc.robot.commands.mouth.*;
import frc.robot.commands.neck.NeckHome;
import frc.robot.commands.neck.NeckMoveDownWithStallDetection;
import frc.robot.commands.neck.NeckMoveSubWithStallDetection;
import frc.robot.subsystems.*;
import frc.robot.interfaces.*;
import frc.robot.sensors.*;
//import frc.robot.auton.sp2.*;


public class StartingPositionTwoLeftThreeNote extends SequentialCommandGroup {

	public StartingPositionTwoLeftThreeNote(RobotContainer container, SwerveDrivetrain drivetrain, Roller roller, Shooter shooter, Neck neck, ICamera object_detection_camera, ICamera apriltag_camera, NoteSensor notesensor, NoteSensor noteSensorTwo){

		addCommands(

			new NeckHome(neck),

			new NeckMoveSubWithStallDetection(neck),

			new ShootNote(shooter, roller),

			new NeckMoveDownWithStallDetection(neck),

			new StartingPositionTwoPickupSecondNote(container, drivetrain, roller, notesensor, noteSensorTwo),

			new NeckMoveSubWithStallDetection(neck),

			new DrivetrainSwerveRelative(drivetrain, container, createShootSecondNoteTrajectory(container)),

			new ShootNote(shooter, roller), 

			new NeckMoveDownWithStallDetection(neck), 

			new StartingPositionTwoPickupLeftThirdNote(container, drivetrain, roller, object_detection_camera, notesensor, noteSensorTwo),

			//new StartingPositionTwoDriveAndTurnToLeftNote(drivetrain, container, object_detection_camera),

			new NeckMoveSubWithStallDetection(neck),

			//new DrivetrainSwerveRelative(drivetrain, container, createShootThirdNoteTrajectory(container)),

			//new StartingPositionTwoShootLeftThirdNote(container, drivetrain, apriltag_camera),

			new DrivetrainSwerveRelative(drivetrain, container, createShootThirdNoteTrajectory(container)),

			new TurnToSpeaker(drivetrain, container, roller, neck, apriltag_camera),

			new ShootNote(shooter, roller),

			new NeckMoveDownWithStallDetection(neck)

			//new DrivetrainSwerveRelative(drivetrain, container, createLeaveAfterShootLeftThirdNoteTrajectory(container))

		); 
  
	}

	public static Trajectory createShootSecondNoteTrajectory(RobotContainer container) {
		// An example trajectory to follow. All units in meters.
		Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
			// Start at the origin facing the -X direction
			new Pose2d(0, 0, Rotation2d.fromDegrees(180)),
			// Pass through these waypoints
			List.of(),
			// End straight ahead of where we started, facing forward
			new Pose2d(AutonConstants.DISTANCE_FROM_STARTING_POSITION_2_TO_SECOND_NOTE_PICKUP, 0, Rotation2d.fromDegrees(180)),
			container.createReverseTrajectoryConfig());

		return trajectory;
	}

	public static Trajectory createShootThirdNoteTrajectory(RobotContainer container) {
		// An example trajectory to follow. All units in meters.
		Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
			// Start at the origin facing the -X direction
			new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
			// Pass through these waypoints
			List.of(),
			// End straight ahead of where we started, facing forward
			new Pose2d(AutonConstants.DISTANCE_FROM_SHOOT_SECOND_NOTE_TO_LEFT_THIRD_NOTE_PICKUP_X, AutonConstants.DISTANCE_FROM_SHOOT_SECOND_NOTE_TO_LEFT_THIRD_NOTE_PICKUP_Y, Rotation2d.fromDegrees(90)),
			container.createReverseTrajectoryConfig());

		return trajectory;
	}	

	/*public static Trajectory createLeaveAfterShootLeftThirdNoteTrajectory(RobotContainer container) {
		// An example trajectory to follow. All units in meters.
		Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
			// Start at the origin facing the -X direction
			new Pose2d(AutonConstants.STARTING_POSITION_2_X_VALUE-AutonConstants.STARTING_POSITION_2_X_VALUE, AutonConstants.STARTING_POSITION_2_Y_VALUE, Rotation2d.fromDegrees(180)),
			// Pass through these waypoints
			List.of(),
			// End straight ahead of where we started, facing forward
			new Pose2d(AutonConstants.DISTANCE_FROM_SHOOT_SECOND_NOTE_TO_LEAVE_X-AutonConstants.STARTING_POSITION_2_X_VALUE, AutonConstants.STARTING_POSITION_2_Y_VALUE-AutonConstants.STARTING_POSITION_2_Y_VALUE, Rotation2d.fromDegrees(180)),
			container.createReverseTrajectoryConfig());

		return trajectory;
	}*/


}