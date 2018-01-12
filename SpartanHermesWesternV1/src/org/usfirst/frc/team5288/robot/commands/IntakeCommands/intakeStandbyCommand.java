package org.usfirst.frc.team5288.robot.commands.IntakeCommands;

import org.usfirst.frc.team5288.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class intakeStandbyCommand extends Command {

	public final String CommandID = "IntakeStandby";
    public intakeStandbyCommand() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.intake);
    }

    // Called just before this Comnmand runs the first time
    protected void initialize() {
    	 Robot.intake.setPower(0);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
     Robot.intake.setPower(0);
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
    }
}
