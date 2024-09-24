package frc.robot.auton.common;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.roller.RollerReleaseShortDistance;
import frc.robot.commands.roller.RollerTimedRelease;
import frc.robot.commands.roller.RollerTimedRoll;
import frc.robot.commands.shooter.ShooterStop;
import frc.robot.commands.shooter.ShooterTimedShootHighNoStop;
import frc.robot.subsystems.*;

public class ShootNote extends SequentialCommandGroup {

	public ShootNote(Shooter shooter, Roller roller) {

		addCommands(

			new ShooterTimedShootHighNoStop(shooter, 0.8),

			//new RollerReleaseShortDistance(roller),

			new RollerTimedRelease(roller, .4),

			new RollerTimedRoll(roller, .4),

			new ShooterStop(shooter),

			new WaitCommand(.25) //.5 //1 // we wait so when we pick up the next note, it doesn't shoot up :)
						
		); 
  
	}
   
}