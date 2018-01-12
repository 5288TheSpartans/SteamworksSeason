
package org.usfirst.frc.team5288.robot;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team5288.robot.autoCommands.driveDistanceStraight;
import org.usfirst.frc.team5288.robot.commands.*;
import org.usfirst.frc.team5288.robot.commands.DriveCommands.DriveStraight;
import org.usfirst.frc.team5288.robot.subsystems.*;

import Accessories.ArduinoComms;
import Accessories.VisionCalculator;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	/**
	 * Robot objects, holds the subsystems, OI, and comms devices.
	 */
	public static final Drivetrain drivetrain = new Drivetrain();
	public static final Intake intake = new Intake();
	public static final Climber climber = new Climber();
	public static final Shooter shooter= new Shooter();
	public static final OI oi = new OI();
	public static final ArduinoComms arduino = new ArduinoComms();
	public static NetworkTable table;
	public static SendableChooser<Command> chooser = new SendableChooser<>();
	public static  VisionCalculator gearCalc =  new VisionCalculator();
	
	public int ultrasonicDistance = 0;
	Command autonomousCommand;
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		CameraServer.getInstance().startAutomaticCapture();
		chooser.addDefault("center gear", new driveDistanceStraight(100));
	    chooser.addObject("DriveStraightInfinite", new DriveStraight());
		SmartDashboard.putData("Auto mode", chooser);
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {
	}

	@Override
	/**
	 * This function is called periodically each time the robot completes a cycle of code during Disabled mode.
	 */
	public void disabledPeriodic() {
		drivetrain.resetEncoders();
		updateSubsystems();	
	}
	
	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {

		autonomousCommand = chooser.getSelected();

		
		

		// schedule the autonomous command (example)
		if (autonomousCommand != null){
			autonomousCommand.start();
		}
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		updateSubsystems();
	}
 
	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand != null)
		{
			autonomousCommand.cancel();
		}
		updateSubsystems();

	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		updateSubsystems();
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		updateSubsystems();
	}
	public void updateSubsystems(){
		drivetrain.update();
		shooter.update();
		intake.update();
		climber.update();
		arduino.changeMode(ArduinoComms.LightsMode.Red);
		gearCalc.Updatedata();
		//System.out.println("Gear peg distance(inches) = " + gearCalc.getDistance());
		//climber.update();
	}
	/**
	 * A private method for parsing Ultrasonic data
	 * 
	 * @param data The data to parse
	 * @return The double value representation the distance to the facing wall in mm
	 */
	/*
	private double parseUltrasonicData(String data)
	{
		//Make sure the most recent data value sent is fully formed
		//If it's not, remove the non-fully formed version
		if(data.lastIndexOf("\r", data.lastIndexOf("R")) == -1)
		{
			data = data.substring(0, data.lastIndexOf("R"));
		}
		
		//Parse the data and return the Double result
		return Double.parseDouble(data.substring(data.lastIndexOf("R") + 1, data.lastIndexOf("\r")));
	}*/
	
	
	
	
	
	
	
	
	
	
}
