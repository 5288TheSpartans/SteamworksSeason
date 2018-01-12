package org.usfirst.frc.team5288.robot.commands.DriveCommands;

import org.usfirst.frc.team5288.robot.Robot;

import Accessories.SpartanPID;
import edu.wpi.first.wpilibj.command.Command;
/**
 *
 */
public class DriveStraight extends Command {
    public final double speed = 0.6;
    public final double gain = 2;
    double error;
    private double initialangle = 0;
    private SpartanPID PID = new SpartanPID(0.009, 0, 0.001, 1/720);
    public DriveStraight() {
    	requires(Robot.drivetrain);
    	initialangle = Robot.drivetrain.getGyroAngle();
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	initialangle = Robot.drivetrain.getGyroAngle();
    	PID.setTarget(0);//PID in error
    }
    
    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	PID.update(Robot.drivetrain.getGyroAngle() - initialangle);
    	error = PID.getOutput();
    	System.out.println("PID INPUT (change in angle):" + (Robot.drivetrain.getGyroAngle() - initialangle));
    	System.out.println("Error: " + error);
    	/*
    	error = gain* (Robot.drivetrain.getGyroAngle() - initialangle);
    	if (error > 1)
    	{
    		error = 0.005;
    	}
    	if (error < -1)
    	{
    		error = -0.005;
    	}*/
    	Robot.drivetrain.setLPower(-speed + error);
    	Robot.drivetrain.setRPower(-speed - error);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.drivetrain.setLPower(0);
    	Robot.drivetrain.setRPower(0);
    
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
