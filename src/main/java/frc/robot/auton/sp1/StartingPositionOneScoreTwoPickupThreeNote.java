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
import frc.robot.commands.shooter.*;
import frc.robot.commands.mouth.*;
import frc.robot.commands.neck.NeckHome;
import frc.robot.commands.neck.NeckMoveDownWithStallDetection;
import frc.robot.commands.neck.NeckMovePodiumWithStallDetection;
import frc.robot.commands.neck.NeckMoveSubWithStallDetection;
import frc.robot.commands.roller.RollerTimedRoll;
import frc.robot.subsystems.*;
import frc.robot.auton.sp1.*;
import frc.robot.interfaces.*;
import frc.robot.sensors.*;


// GP = game piece
// Can be used to place one cube or one cone and either starting position one or two
public class StartingPositionOneScoreTwoPickupThreeNote extends SequentialCommandGroup {

    public StartingPositionOneScoreTwoPickupThreeNote(RobotContainer container, Elevator elevator, SwerveDrivetrain drivetrain, Roller roller, Shooter shooter, Neck neck, ICamera object_detection_camera, ICamera apriltag_camera, NoteSensor notesensor){

        addCommands(

            new NeckHome(neck),

            new NeckMoveSubWithStallDetection(neck),

            new ShootNote(shooter, roller),

            new NeckMoveDownWithStallDetection(neck),

            new DrivetrainTurnUsingCamera(drivetrain, object_detection_camera),

            new StartingPositionOnePickupSecondNote(container, drivetrain, roller, notesensor),

            new NeckMovePodiumWithStallDetection(neck), // check to see if this works later 

			new DrivetrainTurnUsingCamera(drivetrain, apriltag_camera),

            new ShootNote(shooter, roller),

            new NeckMoveDownWithStallDetection(neck),

            new StartingPositionOnePickupThirdNote(container, drivetrain, roller, object_detection_camera, notesensor),

            new NeckMovePodiumWithStallDetection(neck)

        ); 
  
    }

    public Trajectory createShootThirdNoteTrajectory(RobotContainer container) {
		// An example trajectory to follow. All units in meters.
		Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
			// Start at the origin facing the -X direction
			new Pose2d(AutonConstants.DISTANCE_FROM_SECOND_NOTE_PICKUP_TO_THIRD_NOTE_PICKUP_X, AutonConstants.DISTANCE_FROM_SECOND_NOTE_PICKUP_TO_THIRD_NOTE_PICKUP_Y, Rotation2d.fromDegrees(180)),
			// Pass through these waypoints
			List.of(),
			// End straight ahead of where we started, facing forward
			new Pose2d(AutonConstants.DISTANCE_FROM_THIRD_NOTE_PICKUP_TO_SHOOT_THIRD_NOTE_X, AutonConstants.DISTANCE_FROM_THIRD_NOTE_PICKUP_TO_SHOOT_THIRD_NOTE_Y, Rotation2d.fromDegrees(125)),
            container.createReverseTrajectoryConfig());

		return trajectory;
	}
    
    


}