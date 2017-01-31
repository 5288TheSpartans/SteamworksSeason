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
	private double encDiff;//diffEncValue = EncV2 - encv1;
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
		shooterTalon.changeControlMode(CANTalon.TalonControlMode.Voltage);
        /* first choose the sensor */
        shooterTalon.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
        shooterTalon.reverseSensor(false);
        //shooterTalon.configEncoderCodesPerRev(XXX), // if using FeedbackDevice.QuadEncoder
        //shooterTalon.configPotentiometerTurns(XXX), // if using FeedbackDevice.AnalogEncoder or AnalogPot

        /* set the peak and nominal outputs, 12V means full */
        shooterTalon.configNominalOutputVoltage(+0.0f, -0.0f);
        shooterTalon.configPeakOutputVoltage(+12.0f, 0.0f);
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
    		//Calculate encoder difference
    	
    		encLast = encCurrent;
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
    private void setTargetSpeed(){
    	
    	
    }

	public void setForcedPower(){
		
	}
}

