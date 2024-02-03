// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package frc.robot.commands.simpleshooter;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.SimpleShooter;

/**
 *
 */
public class ShooterShoot extends Command {

	private SimpleShooter shooter;

	public ShooterShoot(SimpleShooter shooter) {
		this.shooter = shooter;
		addRequirements(shooter);
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		System.out.println("ShooterShoot: initialize");
		shooter.shoot();
	
	}

}