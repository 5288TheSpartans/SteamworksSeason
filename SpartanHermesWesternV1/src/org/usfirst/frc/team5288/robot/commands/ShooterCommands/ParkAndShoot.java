package org.usfirst.frc.team5288.robot.commands.ShooterCommands;

import org.usfirst.frc.team5288.robot.Robot;

import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ParkAndShoot extends Command {
	final double shooterSpeed = 6000;
	double outputSpeed;
    public ParkAndShoot() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.shooter);
        
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    //Robot.shooter.setTargetSpeed(shooterSpeed);
    	outputSpeed = (Robot.oi.getRightStickThrottle() + 1)/2;
    Robot.shooter.setTargetSpeed(((Robot.oi.getRightStickThrottle() + 1)/2)); 
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	outputSpeed = ((Robot.oi.getRightStickThrottle() + 1)*shooterSpeed)/2;
    	System.out.println("Robot Shooter Throttle Constant :" + (outputSpeed ));
        Robot.shooter.setTargetSpeed(outputSpeed);
       
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
