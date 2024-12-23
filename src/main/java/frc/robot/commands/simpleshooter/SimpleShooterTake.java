
package frc.robot.commands.simpleshooter;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.SimpleShooter;

/**
 *
 */
public class SimpleShooterTake extends Command {

	private SimpleShooter shooter;

	public SimpleShooterTake(SimpleShooter shooter) {
		this.shooter = shooter;
		addRequirements(shooter);
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		System.out.println("ShooterTake: initialize");
		shooter.take();
	}

}
