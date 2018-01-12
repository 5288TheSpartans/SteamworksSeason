
package org.usfirst.frc.team5288.robot;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team5288.robot.autoGroups.LeftGear;
import org.usfirst.frc.team5288.robot.autoGroups.LeftGearB;
import org.usfirst.frc.team5288.robot.autoGroups.RightGear;
import org.usfirst.frc.team5288.robot.autoGroups.RightGearB;
import org.usfirst.frc.team5288.robot.commands.*;
import org.usfirst.frc.team5288.robot.commands.DriveCommands.DriveStraightDistance;
import org.usfirst.frc.team5288.robot.commands.DriveCommands.DriveStraightTime;
import org.usfirst.frc.team5288.robot.commands.DriveCommands.SpotTurnDegrees;
import org.usfirst.frc.team5288.robot.commands.ShooterCommands.ShootDistanceTime;
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
	public static SendableChooser<Integer> chooser = new SendableChooser<>();

	public static SendableChooser<Boolean> allianceChooser = new SendableChooser<>();
	public static  VisionCalculator gearCalc =  new VisionCalculator();
	
	public int ultrasonicDistance = 0;
	public static String alliance = "Blue";
	public static double targetDistance = 0;
	public static double targetTurnAngle = 0;
	Command autonomousCommand;
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		CameraServer.getInstance().startAutomaticCapture();

		chooser.addDefault("Drive Straight only",0);
		chooser.addDefault("Center Gear Only", 1);
		chooser.addObject("Shoot From Wall Only", 2);
		chooser.addObject("Left Gear", 3);
		chooser.addObject("Right Gear", 4);
		chooser.addObject("Do Nothing", 5);
		chooser.addObject("Drive Straight Distance 2.0", 6);
		chooser.addObject("Shoot and Center Gear", 7);
		chooser.addObject("Drive Straight Distance", 8);
		chooser.addObject("Shoot and Left Gear", 9);
		SmartDashboard.putData("Auto mode", chooser);
		allianceChooser.addDefault("Blue", true);
		allianceChooser.addDefault("Red", false);
		SmartDashboard.putData("AllianceChooser",allianceChooser);
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {
		chooser.addDefault("Drive Straight only",0);
		chooser.addDefault("Center Gear Only", 1);
		chooser.addObject("Shoot From Wall Only", 2);
		chooser.addObject("Left Gear", 3);
		chooser.addObject("Right Gear", 4);
		chooser.addObject("Do Nothing", 5);
		chooser.addObject("Drive Straight Distance 2.0", 6);
		chooser.addObject("Shoot and Center Gear", 7);
		chooser.addObject("Drive Straight Distance", 8);

		chooser.addObject("Shoot and Left Gear", 9);
		SmartDashboard.putData("Auto mode", chooser);
		allianceChooser.addDefault("Blue", true);
		allianceChooser.addDefault("Red", false);
		SmartDashboard.putData("AllianceChooser",allianceChooser);
	}

	@Override
	/**
	 * This function is called periodically each time the robot completes a cycle of code during Disabled mode.
	 */
	public void disabledPeriodic() {
		if(allianceChooser.getSelected())
		{
			alliance = "Blue";
		}
		else if(!allianceChooser.getSelected())
		{
			alliance = "Red";
		}
		System.out.println( "Autonomous Selected as of now:"+ chooser.getSelected());
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
		if(allianceChooser.getSelected())
		{
			alliance = "Blue";
		}
		else if(!allianceChooser.getSelected())
		{
			alliance = "Red";
		}
		
		int choice = chooser.getSelected();
		if(choice == 0)
		{
			System.out.println("Running the Autonomous command: Drive Straight");
			SmartDashboard.putString("Running Auto:", "Drive Straight Auto");
			autonomousCommand = new DriveStraightTime(7000);
		}
		else if(choice == 1)
		{
			System.out.println("Running the Autonomous command: Center Gear Only");
			SmartDashboard.putString("Running Auto:", "Center Gear Only");
			autonomousCommand = new DriveStraightTime(4000);
		}
		else if(choice == 2)
		{
			System.out.println("Running the Autonomous command: Shoot From Wall Only");
			SmartDashboard.putString("Running Auto:", "Shoot From Wall Only");
			autonomousCommand = new ShootDistanceTime(160,4000);
		}
		else if(choice == 3)
		{
			System.out.println("Running the Autonomous command: Left Gear");
			SmartDashboard.putString("Running Auto:", "Left Gear");
			if(alliance.equals("Blue"))
			{
				autonomousCommand = new LeftGearB();
			}
			else
			{
				autonomousCommand = new LeftGear();
			}
			
		}
		else if(choice == 4)
		{
			System.out.println("Running the Autonomous command: Right Gear ");
			SmartDashboard.putString("Running Auto:", "Right Gear");
			if(alliance.equals("Blue"))
			{
				autonomousCommand = new RightGearB();
			}
			else
			{
				autonomousCommand = new RightGear();
			}
			
		}
		else if(choice == 5)
		{
			System.out.println("Running the Autonomous command: Do Nothing");
			SmartDashboard.putString("Running Auto:", "Do Nothing");
			autonomousCommand =  new DoNothingTime(4000);
		}
		else if(choice == 6)
		{
			System.out.println("Running the Autonomous command: Drive straight distance");
			SmartDashboard.putString("Running Auto:", "Drive Straight Distance");
			autonomousCommand = new DriveStraightDistance(58);
		}
		else if(choice == 7)
		{
			System.out.println("Running the Autonomous command: Shoot and Center Gear");
			SmartDashboard.putString("Running Auto:", "Shoot and Center Gear");
			autonomousCommand = new SpotTurnDegrees(90);
			autonomousCommand.start();
		}
		else if(choice == 8)
		{
			System.out.println("Running the Autonomous command:EncDriveStraightDistance");
			SmartDashboard.putString("Running Auto:", "EncDriveStraightDistance");
		}
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
		if(allianceChooser.getSelected())
		{
			alliance = "Blue";
		}
		else if(!allianceChooser.getSelected())
		{
			alliance = "Red";
		}
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.		
		if (autonomousCommand != null)
		{
			autonomousCommand.cancel();
		}
		Scheduler.getInstance().removeAll();
		
		Scheduler.getInstance().run();
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
	public void updateSensorInput(){
		
		final double y = 20; // Why didnt I solve this earlier holy crap
		final  double xWallRed= 6;//Fake number
		final double xWallBlue= 18;//Fake number
		double distanceToShoot = 0;
		double realdistance = -1;
		double turnangle = 0;
		if(Robot.getAlliance().equals("Red"))
    	{
    		realdistance = Robot.drivetrain.getUltraSonicDistanceInches() + xWallRed + 19.5;
    		turnangle =-90 - Math.toDegrees(Math.atan(realdistance/-y));
    	}
    	else
    	{
    		realdistance = Robot.drivetrain.getUltraSonicDistanceInches() + xWallBlue + 19.5;
    		turnangle = 90 - Math.toDegrees(Math.atan(realdistance/y));
    		
    	}
    	distanceToShoot = Math.sqrt(

    			(realdistance*realdistance)+(y*y)
    			
    			);
    	targetDistance = distanceToShoot;
    	targetTurnAngle = turnangle;
	}
	public void updateSubsystems(){
		updateSensorInput();
		drivetrain.update();
		shooter.update();
		intake.update();
		climber.update();
		arduino.update();		
		updateSmartDasboards();
		
		//System.out.println("Gear peg distance(inches) = " + gearCalc.getDistance());
		//climber.update();
	}
	public static String getAlliance()
	{
		return alliance;
	}
private void updateSmartDasboards(){

	SmartDashboard.putString("Shooter Speed",""+ Robot.shooter.currentSpeed);
	SmartDashboard.putString("Target Shooter Speed", ""+ Robot.shooter.targetSpeed);
		SmartDashboard.putString("Right Encoder",""+ Robot.drivetrain.getRightDistanceInches());
		SmartDashboard.putString("Left Encoder", ""+ Robot.drivetrain.getLeftDistanceInches());
		SmartDashboard.putString("PID INPUT", Robot.drivetrain.PIDInput);
		SmartDashboard.putString("ShootingTurn", "" +  targetTurnAngle);
		SmartDashboard.putString("ShootingDistance", "" +  targetDistance);
		SmartDashboard.putString("PID OUTPUT", Robot.drivetrain.PIDOutput);
		SmartDashboard.putNumber("Gyro",drivetrain.getGyroAngle());
		SmartDashboard.putNumber("Throttle",drivetrain.getThrottle());
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
