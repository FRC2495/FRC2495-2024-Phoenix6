package frc.robot.auton.sp4;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.RobotContainer;
import frc.robot.auton.AutonConstants;
import frc.robot.auton.common.*;
import frc.robot.commands.drivetrain.*;
import frc.robot.commands.shooter.*;
import frc.robot.interfaces.ICamera;
import frc.robot.commands.mouth.*;
import frc.robot.commands.roller.RollerSuperSmartRoll;
import frc.robot.commands.roller.RollerTimedRoll;
import frc.robot.subsystems.*;
import frc.robot.sensors.*;


public class StartingPositionFourPickupThirdNote extends ParallelCommandGroup {

	public StartingPositionFourPickupThirdNote(RobotContainer container, SwerveDrivetrain drivetrain, ICamera object_detection_camera, Roller roller, NoteSensor notesensor, NoteSensor noteSensorTwo){

		addCommands(

			new RollerSuperSmartRoll(roller, notesensor, noteSensorTwo),

			//new DrivetrainSwerveRelative(drivetrain, container, createPickupThirdNoteTrajectory(container))

			new StartingPositionFourDrivePickupThirdNote(container, drivetrain, object_detection_camera)
			
		); 
  
	}
   
	/*public static Trajectory createPickupThirdNoteTrajectory(RobotContainer container) {
		// An example trajectory to follow. All units in meters.
		Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
			// Start at the origin facing the -X direction
			new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
			// Pass through these waypoints
			List.of(),
			// End straight ahead of where we started, facing forward
			new Pose2d(AutonConstants.DISTANCE_FROM_SHOOT_SECOND_TO_THIRD_PICKUP_X, -AutonConstants.DISTANCE_FROM_SHOOT_SECOND_TO_THIRD_PICKUP_Y, Rotation2d.fromDegrees(-20)),
			container.createTrajectoryConfig());

		return trajectory;
	}*/


}