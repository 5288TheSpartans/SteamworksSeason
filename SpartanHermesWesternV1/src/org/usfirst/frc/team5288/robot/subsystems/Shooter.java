package org.usfirst.frc.team5288.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import org.usfirst.frc.team5288.robot.RobotMap;
import org.usfirst.frc.team5288.robot.commands.ShooterCommands.StopShooter;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

public class Shooter extends Subsystem {

	 

	/*The way to find the necessary turning degree to shoot from beside the wall is to use the
	 * tangent of the total distance from the robot to the center of the goal, x.
	 * Once you have x, you can use the distance of the shooter to the wall (20"), and turn the necessary arctan(x/20)
	 */
	//
	double[] availableDistances = {0,0.5,1,1.5,2,2.5,3,3.5,4,4.5,5,5.5,6,6.5};
	double[] availableSpeeds = {0.1,0.2,0.3,0.36,0.4,0.45,0.5,0.55,0.6,0.65,0.7,0.75,0.8,0.85};

	// TALON SRX and encoder
	 public CANTalon shooterTalon = new CANTalon(RobotMap.ShooterMotor);
	//Encoder Tracking variables
	private double encCurrent = 0;
	//Time tracking
	private double timeLast = 0;
	private double timeCurrent = 0;
	private double timeDiff = 0;
	//Calculate Speed
	private double lastSpeed = 0;
	public double currentSpeed = 0;
	public double targetSpeed = 0;
	//Calculate Acceleration
	private double currentAccel =  0;
	private double lastAccel = 0;
	boolean isVisionTracking = false;
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
        //shooterTalon.setF(0.03140161177014182);
        shooterTalon.setF(0.028929);
        shooterTalon.setP(0);
        shooterTalon.setI(0); 
        shooterTalon.setD(0);
        shooterTalon.changeControlMode(TalonControlMode.Speed);
       // shooterTalon.setControlMode(TalonControlMode.PercentVbus);
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
    		System.out.println("Shooter Speed(RPM): " + currentSpeed);	
	}
    
    private void updateNetworkTables(){
    	    }
    private void updateOutputs(){
    System.out.println("Target shooterSpeed :" + targetSpeed * 5180);
    	shooterTalon.set(targetSpeed);
    	
    }

	public double getSpeedForDistance(double distanceMeters){
		double outputSpeed = -1;
		double distance = distanceMeters;
		for(int i = 0;  i < availableDistances.length; i ++)
		{
			if(availableDistances[i] >= distance)
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
		return outputSpeed;
	}
    public void setTargetSpeed(double newtargetspeed){
        shooterTalon.changeControlMode(TalonControlMode.Speed);
    	targetSpeed = newtargetspeed;
    }
	public void setForcedPower(double power){
		//shooterTalon.changeControlMode(TalonControlMode.Voltage);
	    shooterTalon.changeControlMode(TalonControlMode.PercentVbus);
		shooterTalon.set(power);
	}
}

