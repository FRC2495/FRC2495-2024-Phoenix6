package frc.robot.auton.sp1;

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
import frc.robot.commands.mouth.*;
import frc.robot.commands.roller.RollerTimedRoll;
import frc.robot.subsystems.*;


// GP = game piece
// Can be used to place one cube or one cone and either starting position one or two
public class StartingPositionOnePickupSecondNote extends ParallelCommandGroup {

    public StartingPositionOnePickupSecondNote(RobotContainer container, SwerveDrivetrain drivetrain, Roller roller){

        addCommands(

			new RollerTimedRoll(roller, 3),

            new DrivetrainSwerveRelative(drivetrain, container, createPickupSecondNoteTrajectory(container))
            
        ); 
  
    }
   
    public Trajectory createPickupSecondNoteTrajectory(RobotContainer container) {
		// An example trajectory to follow. All units in meters.
		Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
			// Start at the origin facing the -X direction
			new Pose2d(AutonConstants.STARTING_POSITION_1_X_VALUE-AutonConstants.STARTING_POSITION_1_X_VALUE, AutonConstants.STARTING_POSITION_1_Y_VALUE-AutonConstants.STARTING_POSITION_1_Y_VALUE, Rotation2d.fromDegrees(180)),
			// Pass through these waypoints
			List.of(),
			// End straight ahead of where we started, facing forward
			new Pose2d(AutonConstants.STARTING_POSITION_1_X_VALUE-AutonConstants.DISTANCE_FROM_STARTING_POSITION_1_TO_SECOND_NOTE_PICKUP_X, AutonConstants.STARTING_POSITION_1_Y_VALUE-AutonConstants.STARTING_POSITION_1_Y_VALUE, Rotation2d.fromDegrees(180)),
            container.createTrajectoryConfig());

		return trajectory;
	}


}