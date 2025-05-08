package pedroPathing.actuators;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Actuators {
    public Arm arm;
    public GobildaViper viper;
    public Servos servos;
    public Actuators(String armName, String viperName, String intakeName, String wristName, HardwareMap hwmap) {
        arm = new Arm(armName, hwmap);
        viper = new GobildaViper(viperName, hwmap);
        servos = new Servos(intakeName, wristName, hwmap);
    }
    public void correctActuators() {

    }
}
