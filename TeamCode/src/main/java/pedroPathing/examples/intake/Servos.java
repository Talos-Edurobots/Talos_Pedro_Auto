package pedroPathing.examples.intake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

public class Servos {
    Servo intake, wrist;
    public Servos(String intakeName, String wristName, HardwareMap hwmap) {
        intake = hwmap.get(Servo.class, intakeName);
        wrist  = hwmap.get(Servo.class, wristName);
    }
    public void intakeOpen() {
        intake.setPosition(0);
    }
    public void intakeCollect() {
        intake.setPosition(1);
    }
    public void wristVertical() {
        wrist.setPosition(0.60);
    }
    public void wristHorizontal() {
        wrist.setPosition(0);
    }
}
