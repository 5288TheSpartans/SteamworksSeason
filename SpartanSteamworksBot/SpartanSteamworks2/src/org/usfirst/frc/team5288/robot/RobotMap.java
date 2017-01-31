package org.usfirst.frc.team5288.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	// For example to map the left and right motors, you could define the
	// following variables to use with your drivetrain subsystem.
	// public static int leftMotor = 1;
	// public static int rightMotor = 2 ;
	// Manual Control Constants
	 public static double JOYSAFEZONE = 0.15;
	// Auto Constants 
	
	// Drivetrain
		//Drivetrain motors (PWM)
		public static int LDriveMotor1 = 0;
		public static int LDriveMotor2 = 1;
		public static int RDriveMotor1 = 2;
		public static int RDriveMotor2 = 3;
		// Drivetrain Sensors (D I/O)
		public static int LDriveEncoder1 = 0;
		public static int LDriveEncoder2 = 1;
		public static int RDriveEncoder1 = 2;
		public static int RDriveEncoder2 = 3;
		//Drivetrain PID Constants
		public static double straightP = 0.6;
		public static double straightI = 0;
		public static double straightD = 0;
		public static double rotateP = 0.5;
		public static double rotateI = 0;
		public static double rotateD = 0;
	//Shooter
		//Shooter Motors(PWM)
		public static int ShooterMotor = 4;
		//Shooter Sensors(D I/O)
		public static int ShooterEncoder = 4;
		//Shooter Constants - (KG/M/Rad)
		public static double frictionConstant = 0.0714507874015;
		public static double frictionM =  0.074;
		public static double trueFrictionConstant = frictionM/frictionConstant;
		public static double shooterHeight = 0.8;
	    public static double shooterWeight = 0.0707604097;
	    public static double shooterHardstopSpeed =  15;
	    public static double shooterangle = 1;

	    //Shooter PID Constants 
	    public static double shooterPGain = 0.8;
	    public static double shooterIGain = 0;
	    public static double shooterDGain = 0;
	//Intake
	    //Intake motors
	    public static double intakeMotor = 5;
	    public static double intakePower = 0.5;
	    public static double intakePowerScaler = 13;
	    //This constant will be used by dividing the speed of the drivetrain by the speed of the intake and adding it to the power.
	 //Climber
	    
	 //Arduino
	    public static int ArduinoPort = 1;
	 // public static int Motor
	    
	    
	// If you are using multiple modules, make sure to define both the port
	// number and the module. For example you with a rangefinder:
	// public static int rangefinderPort = 1;
	// public static int rangefinderModule = 1;
}
