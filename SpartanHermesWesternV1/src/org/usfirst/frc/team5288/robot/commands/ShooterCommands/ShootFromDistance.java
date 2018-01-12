package org.usfirst.frc.team5288.robot.commands.ShooterCommands;

import org.usfirst.frc.team5288.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ShootFromDistance extends Command {
	double targetSpeed = 0;
	public ShootFromDistance(double distancemeters) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		targetSpeed = Robot.shooter.getSpeedForDistance(distancemeters);
		requires(Robot.shooter);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.shooter.setTargetSpeed(targetSpeed);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.shooter.setTargetSpeed(targetSpeed);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.shooter.setTargetSpeed(0);

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		
	}
}
