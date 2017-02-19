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
        PulsatingYellowBLue,

        GROOVY
    }

    private I2C _arduino;
    public ArduinoComms() {
        this._arduino = new I2C(I2C.Port.kMXP, RobotMap.ArduinoPort);
    }

    public void changeMode(LightsMode mode) {
        this._arduino.transaction(new byte[] { (byte) mode.ordinal() }, 1, null, 0);
    }

}