package org.usfirst.frc.team5288.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;

import java.util.Scanner;

import org.usfirst.frc.team5288.robot.Robot;
import org.usfirst.frc.team5288.robot.RobotMap;
import org.usfirst.frc.team5288.robot.commands.ShooterCommands.StopShooter;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;

public class Shooter extends Subsystem {

	//Shooter Available distances and corresponding speeds
	double[] availableDistances = {0,0.5,1,1.5,2,2.5,3,3.5,4,4.5,5};
	double[] availableSpeeds = {0,1,2,3,4,5,6,7,8,9,10};

	// TALON SRX and encoder
	public CANTalon shooterTalon = new CANTalon(RobotMap.ShooterMotor);
	//Encoder Tracking variables
	private double currentVoltage = 0;
	private double encLast = 0;
	private double encCurrent = 0;
	//Time tracking
	private double timeLast = 0;
	private double timeCurrent = 0;
	private double timeDiff = 0;
	//Calculate Speed
	private double lastSpeed = 0;
	private double currentSpeed = 0;
	private double targetSpeed = 0;
	//Calculate Acceleration
	private double currentAccel =  0;
	private double lastAccel = 0;
	private double jerk = 0;
	public Shooter(){
		/* first choose the sensor */
		shooterTalon.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		shooterTalon.changeControlMode(CANTalon.TalonControlMode.Speed);
		shooterTalon.reverseSensor(false);
		shooterTalon.reverseOutput(true);
		/* set the peak and nominal outputs, 12V means full */
		shooterTalon.configNominalOutputVoltage(+0.0f, -0.0f);
		shooterTalon.configPeakOutputVoltage(+12.0f, -12.0f);
		/* set th encoder's starting position to 0*/
		/* set closed loop gains in slot0 */
		shooterTalon.setProfile(0);
		shooterTalon.setF(0.1);
		shooterTalon.setP(0.42);
		shooterTalon.setI(0); 
		shooterTalon.setD(0.01);

	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new StopShooter());
	}
	//Any shooting command will call this after setting a speed, allowing the PID to be handled internally.
	public void update()
	{
		updateValues();
		updateOutputs();
		updateNetworkTables();
	}
	/*
	 *This code is used to calculate the speed and acceleration of the shooter wheel. 
	 *The code also calculates the current difference in acceleration, to see what the wheel's
	 *maximum available velocity is.
	 */
	public void updateValues(){
		//Load last Values
		lastSpeed = currentSpeed;
		lastAccel = currentAccel;
		encLast = encCurrent;
		timeLast = timeCurrent;
		//Update Current Values
		timeCurrent = System.currentTimeMillis();
		encCurrent = shooterTalon.getSpeed();//Gets the encoder based speed.
		//Calculate New Values
		//Calculate time difference
		timeCurrent = System.currentTimeMillis();
		timeDiff = timeCurrent - timeLast;
		//calculate Current Speed
		currentSpeed = encCurrent;
		//******Calculate Acceleration and difference in acceleration******
		currentAccel = (lastSpeed- currentSpeed)/timeDiff;
		jerk = (currentAccel - lastAccel)/timeDiff;
		System.out.println("Shooter Speed(RPM): " + currentSpeed);	
	}

	private void updateNetworkTables(){
		/*Robot.table.putNumber("shooterPower",currentVoltage);
    	Robot.table.putNumber("shooterSpeed",currentSpeed);
    	Robot.table.putNumber("shooterAccel",currentAccel);
    	Robot.table.putNumber("shooterJerk",jerk);*/
	}
	private void updateOutputs(){
		System.out.println("Target shooterSpeed :" + targetSpeed);
		shooterTalon.set(targetSpeed);

	}
	public void getShooterSpeedForDistance(){

	}
	public void setTargetSpeed(double newtargetspeed){
		shooterTalon.changeControlMode(CANTalon.TalonControlMode.Speed);
		targetSpeed = newtargetspeed;
	}
	public void setForcedPower(double power){
		shooterTalon.changeControlMode(CANTalon.TalonControlMode.Voltage);
		shooterTalon.set(power);
	}

	public double getSpeedForDistance(double distanceMeters){
		double outputSpeed = 1111;
		double distance = distanceMeters;
		Scanner scan = new Scanner(System.in);
		System.out.println("RUNNING");
		while(true)
		{
			System.out.println("Restarting setup: enter a number");
			distance = scan.nextDouble();
			System.out.println("Searching: ");
			for(int i = 0;  i < availableDistances.length; i ++)
			{
				if(availableDistances[i] < distance)
				{

					System.out.println("The preferred shooting speed is :" +  availableSpeeds[i]);

				}
				else
				{
					outputSpeed = availableSpeeds[i];
					System.out.println("");
					System.out.println("The REAL preferred shooting speed is :" + availableSpeeds[i]);
					System.out.println("This was found at the index of : " + i +  " = " + availableDistances[i]);
					System.out.println("");
					i = availableDistances.length + 1;
				}					
				if (i ==  availableDistances.length- 1 )
				{

					System.out.println("Max Range was hit!:" + availableSpeeds[i]);
					System.out.println("This was found at the index of : " + i +  " = " + availableDistances[i]);
					System.out.println("");
					outputSpeed = availableSpeeds[i];
				}

			}
			System.out.println("Seacrh Complete");
			System.out.println("__________________________________________________________________________________________");
			return outputSpeed;
		}
	}
}
