package org.usfirst.frc.team5288.robot.subsystems;

import org.usfirst.frc.team5288.robot.RobotMap;
import org.usfirst.frc.team5288.robot.commands.ClimberCommands.StopClimber;
import org.usfirst.frc.team5288.robot.commands.IntakeCommands.intakeStandbyCommand;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Climber extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	private VictorSP climberMotorL;
	private VictorSP climberMotorR;
	private double currentPower = 0;
	private boolean isOn = false;
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand (new StopClimber());
    }
    public Climber ()
    {
		climberMotorL = new VictorSP(RobotMap.climberMotorL);
		climberMotorR = new VictorSP(RobotMap.climberMotorR);

    }
    public void setPower(double power)
    {
    	currentPower = power;
    }
    public void update()
    {
    	updateValues();
    	updateOutputs();
    }
    public void updateValues()
    {
		if(currentPower == 0){
			isOn = false;
		}
		else{
			isOn = true;
		}
	}
	public void updateOutputs()
	{	
	climberMotorL.set(-currentPower);
	climberMotorR.set(currentPower);
	}
}

