package frc.robot.auton.sp1;

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
import frc.robot.commands.neck.NeckHome;
import frc.robot.commands.neck.NeckMoveDownWithStallDetection;
import frc.robot.commands.neck.NeckMoveOptimalPositionForShooting;
import frc.robot.commands.neck.NeckMovePodiumWithStallDetection;
import frc.robot.commands.neck.NeckMoveFeedNoteWithStallDetection;
import frc.robot.commands.neck.NeckMoveSubWithStallDetection;
import frc.robot.commands.neck.NeckMoveUsingCamera;
import frc.robot.commands.roller.RollerReleaseShortDistance;
import frc.robot.commands.roller.RollerTimedRoll;
import frc.robot.subsystems.*;
import frc.robot.auton.sp1.*;
import frc.robot.interfaces.*;
import frc.robot.sensors.*;

// when using this path, make sure to position the robot closer to the starting line so note isnt bumped into

public class StartingPositionOneTwoNoteAtMidline extends SequentialCommandGroup {

	public StartingPositionOneTwoNoteAtMidline(RobotContainer container, SwerveDrivetrain drivetrain, Roller roller, Shooter shooter, Neck neck, ICamera object_detection_camera, ICamera apriltag_camera, NoteSensor notesensor, NoteSensor noteSensorTwo){

		addCommands(

			new StartingPositionOneOneNoteAndLeave(container, drivetrain, roller, shooter, neck),

			new StartingPositionOnePickupMidlineNote(container, drivetrain, roller, notesensor, noteSensorTwo),

			//new DrivetrainSwerveRelative(drivetrain, container, createAreaBeforeShootSecondNoteTrajectory(container)),

			//new DrivetrainSwerveRelative(drivetrain, container, createShootSecondNoteTrajectory(container)),

			new StartingPositionOneMoveToSpeakerAndMoveNeck(drivetrain, container, roller, neck, apriltag_camera),

			/*new DrivetrainTurnUsingCamera(drivetrain, apriltag_camera),
			
			new NeckMoveOptimalPositionForShooting(neck, apriltag_camera),*/

			new TurnToSpeaker(drivetrain, container, roller, neck, apriltag_camera),

			new RollerReleaseShortDistance(roller),
			
			new ShootNote(shooter, roller),

			new NeckMoveDownWithStallDetection(neck)

		); 
  
	}

		/*public static Trajectory createAreaBeforeShootSecondNoteTrajectory(RobotContainer container) {
			// An example trajectory to follow. All units in meters.
			Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
				// Start at the origin facing the -X direction
				new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
				// Pass through these waypoints
				List.of(),
				// End straight ahead of where we started, facing forward
				new Pose2d(AutonConstants.DISTANCE_FROM_BEFORE_SECOND_NOTE_PICKUP_TO_MIDLINE_NOTE_PICKUP_X, -AutonConstants.DISTANCE_FROM_BEFORE_SECOND_NOTE_PICKUP_TO_MIDLINE_NOTE_PICKUP_Y, Rotation2d.fromDegrees(0)),
				container.createReverseTrajectoryConfig());
	
			return trajectory;
		}

		public static Trajectory createShootSecondNoteTrajectory(RobotContainer container) {
			// An example trajectory to follow. All units in meters.
			Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
				// Start at the origin facing the -X direction
				new Pose2d(0, 0, Rotation2d.fromDegrees(180)),
				// Pass through these waypoints
				List.of(),
				// End straight ahead of where we started, facing forward
				new Pose2d(AutonConstants.DISTANCE_FROM_BEFORE_SHOOT_SECOND_NOTE_TO_SHOOT_SECOND_NOTE_X, AutonConstants.DISTANCE_FROM_BEFORE_SHOOT_SECOND_NOTE_TO_SHOOT_SECOND_NOTE_Y, Rotation2d.fromDegrees(240)),
				container.createReverseTrajectoryConfig());
	
			return trajectory;
		}*/

}