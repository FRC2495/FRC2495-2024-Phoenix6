// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.subsystems.Shooter;
//import frc.robot.util.Magic;

/**
 *
 */
public class ShooterTimedShootUsingCamera extends WaitCommand {

	private Shooter shooter;

	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
	// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS

	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
	public ShooterTimedShootUsingCamera(Shooter shooter, double timeout) {
		super(timeout);
		this.shooter = shooter;
		addRequirements(shooter);
		
		
		// ControllerBase is not a real subsystem, so no need to reserve it
	}

	// Called just before this Command runs the first time
	/*@Override
	public void initialize() {
		//System.out.println("ShooterTimedShootUsingCamera: initialize");
		//double distance = Robot.camera!=null?Robot.camera.getDistanceToCompositeTargetUsingHorizontalFov():100;
		//double distance = Robot.camera!=null?Robot.camera.getFilteredDistanceToCompositeTarget():100;
		
		//double custom_rpm = Robot.camera!=null?3200.0+(distance-100)*10:3200;

		double verticalOffset = camera!=null?camera.getFilteredVerticalOffsetToCompositeTarget():10;

		double magic_rpm = Magic.getRpm(verticalOffset);

		shooter.shootCustom(magic_rpm);	
	}*/

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
	}

	// Called once after timeout
	@Override
	public void end(boolean interrupted) {
		System.out.println("ShooterTimedShootUsingCamera: end");
		shooter.stop();
	}

}
