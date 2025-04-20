package pedroPathing.examples.actuators;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class actuators {
    public Arm arm;
    public Viper viper;
    public Servos servos;
    public actuators(String armName, String viperName, String intakeName, String wristName, HardwareMap hwmap) {
        arm = new Arm(armName, hwmap);
        viper = new Viper(viperName, hwmap);
        servos = new Servos(intakeName, wristName, hwmap);
    }
    public void correctActuators() {

    }
}
