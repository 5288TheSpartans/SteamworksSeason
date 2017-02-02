package org.usfirst.frc.team5288.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;

import org.usfirst.frc.team5288.robot.RobotMap;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;

public class Shooter extends Subsystem {

	 
	// TALON SRX and encoder
	 CANTalon shooterTalon = new CANTalon(RobotMap.ShooterMotor);
	//Encoder Tracking variables
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
	private double diffAccel = 0;
	public Shooter(){
        /* first choose the sensor */
        shooterTalon.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		shooterTalon.changeControlMode(CANTalon.TalonControlMode.Speed);
        shooterTalon.reverseSensor(false);
        //shooterTalon.configEncoderCodesPerRev(XXX), // if using FeedbackDevice.QuadEncoder
        //shooterTalon.configPotentiometerTurns(XXX), // if using FeedbackDevice.AnalogEncoder or AnalogPot

        /* set the peak and nominal outputs, 12V means full */
        shooterTalon.configNominalOutputVoltage(+0.0f, -0.0f);
        shooterTalon.configPeakOutputVoltage(+12.0f, 0.0f);
        /* set th encoder's starting position to 0*/
        /* set closed loop gains in slot0 */
        shooterTalon.setProfile(0);
        shooterTalon.setF(0.1097);
        shooterTalon.setP(0.22);
        shooterTalon.setI(0); 
        shooterTalon.setD(0);
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    //Any shooting command will call this after setting a speed, allowing the PID to be handled internally.
	public void update()
	{
		updateValues();
		updateOutputs();
	}
/*
 *This code is used to calculate the speed and acceleration of the shooter wheel. 
 *The code also calculates the current difference in acceleration, to see what the wheel's
 *maximum available velocity is.
*/
    public void updateValues(){
    	//Load last Values
    		lastSpeed = currentSpeed;
    		encLast = encCurrent;
    		timeLast = timeCurrent;
    	//Update Current Values
    		timeCurrent = System.currentTimeMillis();
    		encCurrent = shooterTalon.getEncPosition();
    	//Calculate New Values
    		//Calculate time difference
    		timeCurrent = System.currentTimeMillis();
    		timeDiff = timeCurrent - timeLast;
    		//calculate Current Speed
    		currentSpeed = (encCurrent-encLast)/timeDiff;
    		//******Calculate Acceleration and difference in acceleration******

    		currentAccel = (lastSpeed- currentSpeed)/timeDiff;
    		diffAccel = (currentAccel - lastAccel)/timeDiff;

    	    
    }
    private void updateNetworkTables(){
    	
    }
    private void updateOutputs(){
    
    }
    private void setTargetSpeed(double newtargetspeed){
    	targetSpeed = newtargetspeed;
    }

	public void setForcedPower(){
		
	}
}

