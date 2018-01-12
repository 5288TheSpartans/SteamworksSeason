package org.usfirst.frc.team5288.robot.commands.DriveCommands;

import org.usfirst.frc.team5288.robot.Robot;

import Accessories.SpartanPID;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SpotTurnDegrees extends Command {
	public final double speed = 0.5;
	public final double gain = 2;
	private double currentTurn = 0;
	private double initialangle = 0;
	private double targetAngle = 0;
	private double PIDOUTPUT = 0;
	private SpartanPID PID = new SpartanPID(0.02, 0.0095, 0.09, 0);
	private double startTime = 0;
	public SpotTurnDegrees(double turn) {
		requires(Robot.drivetrain);
		targetAngle = turn;
		initialangle = Robot.drivetrain.getGyroAngle();
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		startTime = System.currentTimeMillis();
		initialangle = Robot.drivetrain.getGyroAngle();
		PID.setTarget(targetAngle);//PID in turn
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		currentTurn = Robot.drivetrain.getGyroAngle() - initialangle;
		PID.update(currentTurn);
		Robot.drivetrain.PIDInput = " " +currentTurn;
		Robot.drivetrain.PIDOutput = " "+ PIDOUTPUT;
		
		System.out.println("TURN PID INPUT : " + currentTurn);
		System.out.println("TURN PID Target : " + targetAngle);
		System.out.println("TURN PID OUTPUT: " + PIDOUTPUT);
		if(currentTurn > targetAngle - 1 && currentTurn < targetAngle + 1)
		{
			PIDOUTPUT = 0;
		}
		else
		{
			PIDOUTPUT = PID.getOutput();
		}
		Robot.drivetrain.setLPower(-speed*PIDOUTPUT);
		Robot.drivetrain.setRPower(speed*PIDOUTPUT);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		System.out.println("Checking Turn To see if finished");
		if(Math.abs(targetAngle)<= Math.abs(currentTurn) +1)
		{
			return true;
		}
		if(System.currentTimeMillis()>= startTime + 2500)
		{
			return true;
			System.err.println("Command did not achieve goal, ended through time.");
		}
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.drivetrain.setLPower(0);
		Robot.drivetrain.setRPower(0);
	}
	// Called when another command which requires one or more of the same
	// subsystems is scheduled to runa
	protected void interrupted() {
	}

}