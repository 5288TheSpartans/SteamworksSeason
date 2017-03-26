package Accessories;

import edu.wpi.first.wpilibj.I2C;
import org.usfirst.frc.team5288.robot.RobotMap;
public class ArduinoComms {
	int counter = 0;
	public enum LightsMode {
		Yellow,
		PulsatingYellow,

		Red,
		PulsatingRed,
		PulsatingYellowRed,

		Blue,
		PulsatingBlue,
		PulsatingYellowBlue,

		GROOVY
	}
	private int currentIndex = 0;
	private LightsMode currentMode = LightsMode.Yellow;
	private I2C _arduino;
	public void nextColor(){
		if(currentIndex ==  7)
		{
			currentIndex = 0;
		}
		else{
			currentIndex ++;
		}
		switch (currentIndex){
		case 0:
			currentMode = LightsMode.Yellow; 
			break;
		case 1:
			currentMode = LightsMode.PulsatingYellow;
			break;
		case 2:
			currentMode = LightsMode.Red;
			break;
		case 3:
			currentMode = LightsMode.PulsatingRed;
			break;
		case 4:
			currentMode = LightsMode.PulsatingYellowRed;
			break;
		case 5:
			currentMode = LightsMode.Blue;
			break;
		case 6:
			currentMode = LightsMode.PulsatingBlue;
			break;
		case 7:
			currentMode = LightsMode.PulsatingYellowBlue;
			break;
		case 8:
			currentMode = LightsMode.GROOVY;
			break;
		
		default:	
			currentMode = LightsMode.PulsatingYellow;
			break;
		}

		changeMode(currentMode);

	}

	public void lastColor(){
		if(currentIndex ==  0)
		{
			currentIndex = 7;
		}
		else{
			currentIndex ++;
		}
		switch (currentIndex){
		case 0:
			currentMode = LightsMode.Yellow; 
			break;
		case 1:
			currentMode = LightsMode.PulsatingYellow;
			break;
		case 2:
			currentMode = LightsMode.Red;
			break;
		case 3:
			currentMode = LightsMode.PulsatingRed;
			break;
		case 4:
			currentMode = LightsMode.PulsatingYellowRed;
			break;
		case 5:
			currentMode = LightsMode.Blue;
			break;
		case 6:
			
			currentMode = LightsMode.PulsatingBlue;
			break;
		case 7:
			currentMode = LightsMode.PulsatingYellowBlue;
			break;
		case 8:
			currentMode = LightsMode.GROOVY;
			break;
		
		default:	
			currentMode = LightsMode.PulsatingYellow;
			break;
		}

		changeMode(currentMode);
	}
	public ArduinoComms() {
		this._arduino = new I2C(I2C.Port.kMXP, RobotMap.ArduinoPort);
	}

	public void changeMode(LightsMode mode) {
		System.out.println("Arduino Color Changed to:" + currentMode);
		this._arduino.transaction(new byte[] { (byte) mode.ordinal() }, 1, null, 0);
	}

}