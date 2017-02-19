package org.usfirst.frc.team5288.robot;

import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;

import org.usfirst.frc.team5288.robot.commands.*;
/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    // One type of button is a joystick button which is any button on a joystick.
    // You create one by telling it which joystick it's on and which button
    // number it is.
    // Joystick stick = new Joystick(port);
    // Button button = new JoystickButton(stick, buttonNumber);
    Joystick joystickLeft = new Joystick(0);    //left joystick is in port 1
    Joystick joystickRight = new Joystick(1);    //right joystick is in port 0
    Joystick xbox = new Joystick(2);
    
 	private JoystickButton btnL1 = new JoystickButton(joystickLeft,1),
 					btnLowThrottle = new JoystickButton(joystickLeft,2),
					btnHighThrottle = new JoystickButton(joystickLeft,3),
					btnDriveToPosition = new JoystickButton(joystickLeft,4),
					btnL5 = new JoystickButton(joystickLeft,5),
					btnL6 = new JoystickButton(joystickLeft,6),
 					btnL7 = new JoystickButton(joystickLeft,7),
 					btnL8 = new JoystickButton(joystickLeft,8),
 					btnL9 = new JoystickButton(joystickLeft,9),
 				btnR1 = new JoystickButton(joystickRight,1),
				btnR2 = new JoystickButton(joystickRight,2),
				btnR3 = new JoystickButton(joystickRight,3),
				btnR4 = new JoystickButton(joystickRight,4),
				btnR5 = new JoystickButton(joystickRight,5),
				btnR6 = new JoystickButton(joystickRight,6),
				btnR7 = new JoystickButton(joystickRight,7),
				btnR8 = new JoystickButton(joystickRight,8),
				btnR9 = new JoystickButton(joystickRight,9),
 				btnR10 = new JoystickButton(joystickRight,10),
 		 		btnR11 = new JoystickButton(joystickRight,1); 
 	private JoystickButton xBoxB = new JoystickButton(xbox,0),
			 xboxA= new JoystickButton(xbox,1),
			xboxY = new JoystickButton(xbox,2),
			xboxX = new JoystickButton(xbox,3),
			xboxLB = new JoystickButton(xbox,6),
			xboxRB = new JoystickButton(xbox,7),
			xboxStart = new JoystickButton(xbox,8);
			
		 	

    public OI()
    {
   
    //Shooter Commands
    btnR3.toggleWhenPressed(new ParkAndShoot());
    btnR4.toggleWhenPressed(new IntakeBalls());
   // btnR1.toggleWhenPressed(new DriveStraight());
    // There are a few additional built in buttons you can use. Additionally,
    // by subclassing Button you can create custom triggers and bind those to
    // commands the same as any other Button.

    //// TRIGGERING COMMANDS WITH BUTTONS
    // Once you have a button, it's trivial to bind it to a button in one of
    // three ways:
    
    // Start the command when the button is pressed and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenPressed(new ExampleCommand());
    
    // Run the command while the button is being held down and interrupt it once
    // the button is released.
    // button.whileHeld(new ExampleCommand());
    
    // Start the command when the button is released  and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenReleased(new ExampleCommand());
    	//btnDriveToPosition.whenPressed();
    	//btnTurnAngle.whenPressed();
    	//btn
    	//Shooter Buttons
    	/*btnL3.whileHeld(new ManualShooterUp());
    	 * btnL2.whileHeld(new ManualShooterDown());
    	 * btnL1.whenPressed(new IntakeBall());
    	 * btnR1.whileHeld(new PrepareBall());
    	 * btnR2.whileHeld(new ShootBall());
    	 * btnR9.whileHeld(new DriveStraight());
    	 * btnx1.whileHeld(new AimToShoot());
    	 * btn
   */
    	
    //-----------------------------Code for returning Joystick Axis--------------------------
    
    }
    public double getXLY()
    {
    	return (-xbox.getRawAxis(1));
    }
    public double getXLX()
    {
    	return (xbox.getRawAxis(0));
    }
    public double getXRY()
    {
    	return (-xbox.getRawAxis(5));
    }
    public double getXRX()
    {
    	return (xbox.getRawAxis(4));
    }
    public double getLeftStickY()
    {
     
    	return(joystickLeft.getY());//Returns the chosen axis specified by code.
    }
    public double getLeftStickX()
    {
    	return(joystickLeft.getX());//Returns the chosen axis specified by code.
    }
    //Right Joystick
    public double getRightStickY()
    {
    	return(joystickRight.getY());//Returns the chosen axis specified by code.
    }
    public double getRightStickX()
    {
    	return(joystickRight.getX());
    }
    public double getRightStickThrottle()
    {
    	return(joystickRight.getThrottle());
    }
}	

