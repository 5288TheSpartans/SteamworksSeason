package org.usfirst.frc.team5288.robot.commands.DriveCommands;

import org.usfirst.frc.team5288.robot.Robot;
import org.usfirst.frc.team5288.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ManualDrive extends Command {

	public ManualDrive() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires();
		requires(Robot.drivetrain);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.drivetrain.setLPower(0);// Stops the robot immediately
		Robot.drivetrain.setRPower(0);//Stops
	}
	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		/*The code here is organised so that the execute Checks if a directional
		 * button is being held down, and if it is, it turns the robot in that
		 * direction. If not, the manual drive style function of the code where 
		 * raw inputs from both of the joysticks are used to control their
		 * respective sides of the robot's drivetrain, outputting power equal to
		 * the raw input multiplied by the throttle multiplier, allowing for 
		 * extremely precise movements.
		 * ************THE ABOVE HAS BEEN DEPRECATED FOR THE 2017 BOT
		 */
		
		
		//Get Values
			
		//
		//Pure manual Joysticks drive 
		//Left Side 
		if(Robot.oi.getLeftStickX() <= RobotMap.JOYSAFEZONE || Robot.oi.getLeftStickX() >= -RobotMap.JOYSAFEZONE)//checks if the joystick is in RobotMap.JOYSAFEZONE
		{
			Robot.drivetrain.setLPower(Robot.oi.getLeftStickY());
		}
		else// if the joystick Y value is in the RobotMap.JOYSAFEZONE, sets the robot's left motor to 0 output
		{
			Robot.drivetrain.setLPower(0);
		}
		//Right Side
		if(Robot.oi.getRightStickY() >= RobotMap.JOYSAFEZONE || Robot.oi.getRightStickY() <= -RobotMap.JOYSAFEZONE)//checks if the joystick is in RobotMap.JOYSAFEZONE
		{
			//sets the robots' right speed through a method declared in the drive subystem
			Robot.drivetrain.setRPower(Robot.oi.getRightStickY());
		}
		else
		{
			Robot.drivetrain.setRPower(0);
		}

	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		Robot.drivetrain.setRPower(0);
		Robot.drivetrain.setLPower(0);
	}
}
