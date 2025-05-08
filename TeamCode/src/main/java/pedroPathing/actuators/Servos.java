package pedroPathing.actuators;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Servos {
    public Servo intake, wrist;
    public boolean hardware;
    public double intakePos;
    public double wristPos;
    public Servos(String intakeName, String wristName, HardwareMap hwmap) {
        init(intakeName, wristName, hwmap);
        hardware = true;
    }
    public Servos(String intakeName, String wristName, HardwareMap hwmap, boolean hardware) {
        if (hardware) {
            init(intakeName, wristName, hwmap);
            this.hardware = true;
        }
        else {
            hardware = false;
        }
    }
    public void init(String intakeName, String wristName, HardwareMap hwmap) {
        intake = hwmap.get(Servo.class, intakeName);
        wrist  = hwmap.get(Servo.class, wristName);
    }
    public void intakeOpen() {
        if (hardware) {
            intake.setPosition(0);
            intakePos = 0;
        }
    }
    public void intakeCollect() {
        if (hardware) {
            intake.setPosition(1);
            intakePos = 1;
        }
    }
    public void wristVertical() {
        if (hardware) {
            wrist.setPosition(.6);
            wristPos = .6;
        }
    }
    public void wristHorizontal() {
        if (hardware) {
            wrist.setPosition(0);
            wristPos = 0;
        }
    }
    public void setIntakePosition(double pos) {
        intakePos = pos;
        if (!hardware) {
            return;
        }
        intake.setPosition(pos);
    }
    public void setWristPosition(double pos) {
        intakePos = pos;
        if (!hardware) {
            return;
        }
        wrist.setPosition(pos);
    }
    public double getIntakePosition() {
        if (hardware) {
            return intake.getPosition();
        }
        return intakePos;
    }
    public double getWristPosition() {
        if (hardware) {
            return wrist.getPosition();
        }
        return wristPos;
    }
}
