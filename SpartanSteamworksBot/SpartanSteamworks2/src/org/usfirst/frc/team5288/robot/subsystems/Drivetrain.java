package org.usfirst.frc.team5288.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.VictorSP;

import org.usfirst.frc.team5288.robot.Robot;
import org.usfirst.frc.team5288.robot.RobotMap;
import org.usfirst.frc.team5288.robot.commands.ManualDrive;
import edu.wpi.first.wpilibj.Encoder;
/**
 *
 */
public class Drivetrain extends Subsystem {
	
	/*
	 * Drive Kinematics math
	 * a = (v1-v2)/2
	 * 
	 */
	//**DRIVETRAIN CONSTANTS**
	public final double wheelRadius = 0.0508; //meters
	public final double wheelcirc = 2*Math.PI*wheelRadius; 
	public final double topSpeed =  3.048; // meters per second
	public final double vfeedForward = 1/3.048; //meters per second
	public final double kp = 0.04;
	private double encPPR = 500;
	//*******************MOTOR CONTROLLER OBJECTS**************
	//These Motor controller objects will always be synced in pairs of output.
	private VictorSP lmotor1 = new VictorSP(RobotMap.LDriveMotor1);//Left Gearbox Motor #
	private VictorSP lmotor2 = new VictorSP(RobotMap.LDriveMotor2);//Left Gearbox Motor #
	private VictorSP rmotor1 = new VictorSP(RobotMap.RDriveMotor1);//Right Gearbox Motor #1
	private VictorSP rmotor2 = new VictorSP(RobotMap.RDriveMotor2);//Right Gearbox Motor #2
	private boolean isBrakeMode = true;
	
	//**Drive Variables**
	private double throttle = 1;
	private double lPower = 0;//Raw Power percentage being output to the left gearbox.
	private double rPower = 0;//Raw Power percentage being output to the right gearbox.
	
	//**ENCODER VARIABLES**
	private Encoder rEncoder = new Encoder(RobotMap.RDriveEncoder1,RobotMap.RDriveEncoder2,false,Encoder.EncodingType.k4X);
	private Encoder lEncoder = new Encoder(RobotMap.LDriveEncoder1,RobotMap.LDriveEncoder2,true,Encoder.EncodingType.k4X);	
	
	// Distance = Velocity1*time + 1/2*Acceleration*time^2 
	//**SPEED CALCULATION BASED VARIABLES**	//Encoder Tracking variables
		//Left
	private double lastSpeedL = 0;
	private double lastAccelL = 0;
	private double currentAccelL = 0;
	private double diffAccelL = 0;
	private double currentSpeedL = 0;
	private double targetSpeedL = 0;
	private double encLastL = 0;
	private double encCurrentL = 0;
	private double encDiffL = 0;
		//Right
	private double lastAccelR = 0;
	private double currentAccelR = 0;
	private double diffAccelR = 0;
	private double lastSpeedR = 0;
	private double currentSpeedR = 0;
	private double targetSpeedR = 0;
	private double encLastR = 0;
	private double encCurrentR = 0;
	private double encDiffR = 0;
		//Time tracking
	private double timeLast = 0;
	private double timeCurrent = 0;
	private double timeDiff = 0;
	//*Built-In State Machine*
	//The state machine ensures that the drivetrain either A: Runs it's P.I.D controls,
	// or B: runs off of the raw data being supplied to it, one of the two will always occur due to the default command.
	public enum drivestates  {PID,MANUAL, AUTOPID};
	private drivestates currentState = drivestates.PID;
	
	//******************************Instantiates the DRIVETRAIN SUBCLASS***************************
	public void Drivetrain (){
		rEncoder.setDistancePerPulse(wheelcirc/encPPR);
	}
	
	//******************************DriveTrain Methods and Procedures******************************  

	public void initDefaultCommand() {
		setDefaultCommand(new ManualDrive());
	}
	public void update()
	{
		updateSensorVals();
		updateOutputs();
	}
	public void updateSensorVals(){
		//Load last Values
		lastSpeedL = currentSpeedL;
		lastSpeedR = currentSpeedR;
		lastAccelL = currentAccelL;
		lastAccelR = currentAccelR;
		encLastR = encCurrentR;
		encLastL = encCurrentL;
		timeLast = timeCurrent;

		//Update Current Values
		timeCurrent = System.currentTimeMillis();
		encCurrentL = lEncoder.getDistance();
		encCurrentR = rEncoder.getDistance();

		//******Calculate New Values*******
		timeDiff = timeCurrent - timeLast;//Calculate time difference
		encDiffL = encCurrentL - encLastL;//Calculate encoder difference
		encDiffR = encCurrentR - encLastR;//Calculate encoder difference
		//Calculate The Current speed
		currentSpeedL = encDiffL/timeDiff;
		currentSpeedR = encDiffR/timeDiff;
		//******Calculate Acceleration and difference in acceleration******
		currentAccelL = (lastSpeedL - currentSpeedL)/timeDiff;
		currentAccelR = (lastSpeedR- currentSpeedR)/timeDiff;
		diffAccelL = (currentAccelL - lastAccelL)/timeDiff;
		diffAccelR = (currentAccelR - lastAccelR)/timeDiff;

	}
	private void updateOutputs(){

	}
	private void setTargetSpeeds(double Left, double Right)//Sets the PID to control the robot.
	{

	}
	//**Throttle is used to maximize the output potential by the MANUAL driver
	//TODO: Implement Throttle
	private void OutputToMotors(double pwrLeft, double pwrRight){
		lmotor1.set(pwrLeft);
		lmotor2.set(pwrLeft);
		rmotor1.set(pwrRight);
		rmotor2.set(pwrRight);
		System.out.println("Outputted power to Leftdrive:" +  pwrLeft);
		System.out.println("Outputted power to Leftdrive:" +  pwrRight);
	}
	public void setThrottle(double newThrottle)
	{
		throttle = newThrottle;
	}
	public double getThrottle(){
		return throttle;
	}
	public void setLPower(double power)
	{
		currentState = drivestates.MANUAL;
		lPower = power;
	}
	public void setRPower(double power)
	{
		currentState = drivestates.MANUAL;
		rPower = power;
	}
	public double getLeftDistanceMeters()
	{
		return lEncoder.getDistance();
	}
	public double getRughtDistanceMeters()
	{
		return rEncoder.getDistance();
	}
	public void resetEncoders()//Should not need to be used except when changing control periods.
	{
		lEncoder.reset();
		rEncoder.reset();
	}

    private double rotationsTometers(double rotations) {
        return rotations * (wheelRadius * Math.PI);
    }

    private double rpmToInchesPerSecond(double rpm) {
        return rotationsToInches(rpm) / 60;
    }

    private double metersToRotations(double meters) {
        return meters / (2* wheelRadius * Math.PI);
    }

    private double metersPerSecondToRpm(double meters_per_second) {
        return metersToRotations(meters_per_second) * 60;
    
    }

    public void setBrakeMode(boolean on) {
        if (isBrakeMode != on) {
            lmotor1.enableBrakeMode(on);
            lmotor2.enableBrakeMode(on);
            rmotor1.enableBrakeMode(on);
            rmotor2.enableBrakeMode(on);
            isBrakeMode = on;
        }
    }
}

