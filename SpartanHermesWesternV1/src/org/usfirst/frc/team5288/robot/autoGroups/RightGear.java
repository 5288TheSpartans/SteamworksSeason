package org.usfirst.frc.team5288.robot.autoGroups;

import org.usfirst.frc.team5288.robot.commands.DoNothingTime;
import org.usfirst.frc.team5288.robot.commands.DriveCommands.BackDriveStraightTime;
import org.usfirst.frc.team5288.robot.commands.DriveCommands.DriveStraightDistance;
import org.usfirst.frc.team5288.robot.commands.DriveCommands.DriveStraightTime;
import org.usfirst.frc.team5288.robot.commands.DriveCommands.SpotTurnDegrees;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class RightGear extends CommandGroup {

    public RightGear() {
        // Add Commands here:

    	addSequential(new DriveStraightDistance(75));
    	addSequential(new DoNothingTime(1000));
    	addSequential(new SpotTurnDegrees(63));
    	addSequential(new DoNothingTime(1000));
    	addSequential(new DriveStraightTime(2000));
    	addSequential(new DoNothingTime(4000));
    	addSequential(new BackDriveStraightTime(1000));
    	addSequential(new DoNothingTime(1000));
    	addSequential(new SpotTurnDegrees(-63));
    	addSequential(new DoNothingTime(500));
    	addSequential(new DriveStraightDistance(100));
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
    }
}
