package Accessories;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.command.Subsystem;

import org.usfirst.frc.team5288.robot.RobotMap;
public class ArduinoComms extends Subsystem{
	public enum LightsMode {
		Yellow,
		Green,
		Red,
		Blue,

	}
	private LightsMode currentMode = LightsMode.Yellow;
	private DigitalOutput arduinoA;
	private DigitalOutput arduinoB;
	public void setColor(LightsMode newMode){
		currentMode = newMode;
		changeMode(currentMode);
	}
	public ArduinoComms() {		
		arduinoA = new DigitalOutput(8);
		arduinoB = new DigitalOutput(9);

	}
	public void update()
	{
		changeMode(currentMode);
	}
	private void changeMode(LightsMode mode) {
		switch (currentMode){
		case Yellow : 

			arduinoA.set(true);
			arduinoB.set(false);
			break;
		case Red:

			arduinoA.set(false);
			arduinoB.set(true);
			break;
		case Blue:
			arduinoA.set(true);
			arduinoB.set(true);
			break;
		case Green:
			arduinoA.set(false);
			arduinoB.set(false);
			break;
		default:	
			arduinoA.set(false);
			arduinoB.set(false);
			break;
		}
	}
	@Override
	protected void initDefaultCommand() {
		
	}

}