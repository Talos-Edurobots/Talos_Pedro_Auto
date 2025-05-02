package pedroPathing.actuators;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Servos {
    public Servo intake, wrist;
    public boolean hardware;
    public Servos(String intakeName, String wristName, HardwareMap hwmap) {
        intake = hwmap.get(Servo.class, intakeName);
        wrist  = hwmap.get(Servo.class, wristName);
    }
    public Servos(String intakeName, String wristName, HardwareMap hwmap, boolean hardware) {
        if (hardware) {
            new Servos(intakeName, wristName, hwmap);
        }
        else {
            hardware = false;
        }
    }
    public void intakeOpen() {
        if (hardware) {
            intake.setPosition(0);
        }
    }
    public void intakeCollect() {
        if (hardware) {
            intake.setPosition(1);
        }
    }
    public void wristVertical() {
        if (hardware) {
            wrist.setPosition(.6);
        }
    }
    public void wristHorizontal() {
        if (hardware) {
            wrist.setPosition(0);
        }
    }
}
