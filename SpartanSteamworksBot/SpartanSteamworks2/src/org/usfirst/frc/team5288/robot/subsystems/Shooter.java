package org.usfirst.frc.team5288.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;

import org.usfirst.frc.team5288.robot.Robot;
import org.usfirst.frc.team5288.robot.RobotMap;
import org.usfirst.frc.team5288.robot.commands.StopShooter;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;

public class Shooter extends Subsystem {

	 
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
    		encCurrent = shooterTalon.get();//Gets the encoder based speed.
    	//Calculate New Values
    		//Calculate time difference
    		timeCurrent = System.currentTimeMillis();
    		timeDiff = timeCurrent - timeLast;
    		//calculate Current Speed
    		currentSpeed = encCurrent;
    		//******Calculate Acceleration and difference in acceleration******
    		currentAccel = (lastSpeed- currentSpeed)/timeDiff;
    		jerk = (currentAccel - lastAccel)/timeDiff;
    		System.out.println("Speed, Accel, Jerk :" + currentSpeed + " , " + currentAccel + " , " + jerk);	
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
    public void setTargetSpeed(double newtargetspeed){
        shooterTalon.changeControlMode(CANTalon.TalonControlMode.Speed);
    	targetSpeed = newtargetspeed;
    }

	public void setForcedPower(double power){
		shooterTalon.changeControlMode(CANTalon.TalonControlMode.Voltage);
		shooterTalon.set(power);
	}
}

