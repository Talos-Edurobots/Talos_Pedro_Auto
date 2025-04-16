package pedroPathing.examples.actuators;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Arm {
    public DcMotor arm;
    public Telemetry tel;
    private int pos; // in ticks
    private double armTicksPerDegree = (28 // number of encoder ticks per rotation of the bare motor
                        * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                        * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                        * 1/360.0);
    int armDegreesToTicks(double degrees) {
        return (int) (degrees * armTicksPerDegree);
    }
    int armTicksToDegrees(int ticks) {
        return (int) (ticks / armTicksPerDegree);
    }

    public Arm(String name, HardwareMap hwmap, Telemetry tel) {
        this.tel = tel;
        arm = hwmap.get(DcMotor.class, name); //the arm motor
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ((DcMotorEx) arm).setCurrentAlert(5, CurrentUnit.AMPS);
        arm.setTargetPosition(0);
        arm.setDirection(DcMotor.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void setPositionTicks(int pos) {
        this.pos = pos;
    }
    public void setPositionDegrees(int position) {
        pos = armDegreesToTicks(position);
    }
    public int getPositionDegrees() {
        return armTicksToDegrees(pos);
    }
    public void runArm(boolean sequentially) {
        arm.setTargetPosition(pos);
        arm.setPower(1);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION); // we finally run the arm motor
        if (!sequentially) {
            return;
        }
        while (arm.isBusy()) {
            continue;
        }
    }
}
