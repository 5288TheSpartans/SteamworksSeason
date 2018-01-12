package org.usfirst.frc.team5288.robot.commands;

import org.usfirst.frc.team5288.robot.Robot;
import org.usfirst.frc.team5288.robot.commands.DriveCommands.SpotTurnDegrees;
import org.usfirst.frc.team5288.robot.commands.ShooterCommands.ShootFromDistance;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class ShootAgainstWall extends CommandGroup {

    public ShootAgainstWall() {
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.
    	addSequential(new SpotTurnDegrees(Robot.targetTurnAngle));
    	addSequential(new ShootFromDistance(Robot.targetDistance));

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
