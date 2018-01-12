package org.usfirst.frc.team5288.robot.autoGroups;

import org.usfirst.frc.team5288.robot.Robot;
import org.usfirst.frc.team5288.robot.commands.DoNothingTime;
import org.usfirst.frc.team5288.robot.commands.DriveCommands.DriveStraightDistance;
import org.usfirst.frc.team5288.robot.commands.DriveCommands.DriveStraightTime;
import org.usfirst.frc.team5288.robot.commands.DriveCommands.SpotTurnDegrees;
import org.usfirst.frc.team5288.robot.commands.ShooterCommands.ShootDistanceTime;
import org.usfirst.frc.team5288.robot.commands.ShooterCommands.ShootFromDistance;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class ShootAndCenterGear extends CommandGroup {

    public ShootAndCenterGear(String Colour) {
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order
    	//addSequential(new ShootFromDistance(3));
    	addSequential(new ShootDistanceTime(3,2000));
    	if(Colour.equals("blue"))
    	{
    		addSequential(new SpotTurnDegrees(97));
    	}
    	else
    	{
    		addSequential(new SpotTurnDegrees(-97));
    	}
    	addSequential(new DriveStraightTime(2800));
    	addSequential(new DoNothingTime(3));
    	addSequential(new DriveStraightDistance(-3.5));
    	//addSequential(new SpotTurnDegrees(-90));
    	//addSequential(new DriveStraightTime(1500));
    	//addSequential(new SpotTurnDegrees(90));
    	//addSequential(new DriveStraightTime(4));
        
    	// To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
    	requires(Robot.drivetrain);
    	requires(Robot.shooter);
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
    }
}
