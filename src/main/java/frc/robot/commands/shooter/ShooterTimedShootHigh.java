package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.subsystems.Shooter;


/**
 * Add your docs here.
 */
public class ShooterTimedShootHigh extends WaitCommand {
	/**
	 * Add your docs here.
	 */
	private Shooter shooter;

	public ShooterTimedShootHigh(Shooter shooter, double timeout) {
		super(timeout);
		this.shooter = shooter;
		addRequirements(shooter);
		
		
		// ControllerBase is not a real subsystem, so no need to reserve it
	}


	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		System.out.println("ShooterTimedShootHigh: initialize");
		super.initialize();
		shooter.shootHigh();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
	}

	// Called once after timeout
	@Override
	public void end(boolean interrupted) {
		System.out.println("ShooterTimedShootHigh: end");
		shooter.stop();
		super.end(interrupted);
	}
}
