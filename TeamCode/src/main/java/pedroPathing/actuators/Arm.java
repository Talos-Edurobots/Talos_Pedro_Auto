package pedroPathing.actuators;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


public class Arm {
    public DcMotor arm;
    public DcMotor.RunMode runMode = DcMotor.RunMode.RUN_TO_POSITION;
    public boolean hardware;
    private boolean relaxed;
    //    public Telemetry tel;
    private int pos; // in ticks
    private int velocity = 3000;
    private static double armTicksPerDegree = (28 // number of encoder ticks per rotation of the bare motor
            * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
            * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
            * 1 / 360.0);

    public static int degreesToTicks(double degrees) {
        return (int) (degrees * armTicksPerDegree);
    }

    public static double ticksToDegrees(int ticks) {
        return (int) (ticks / armTicksPerDegree);
    }

    public Arm(String name, @NonNull HardwareMap hwmap) {
        init(name, hwmap);
    }

    public Arm(String name, HardwareMap hwmap, boolean hardware) {
        if (hardware) {
            init(name, hwmap);
        } else {
            hardware = false;
        }
    }

    public void init(String name, @NonNull HardwareMap hwmap) {
        arm = hwmap.get(DcMotor.class, name); //the arm motor
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware = true;
        ((DcMotorEx) arm).setCurrentAlert(5, CurrentUnit.AMPS);
        arm.setTargetPosition(0);
        arm.setDirection(DcMotor.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setPositionTicks(int pos) {
        this.pos = pos;
    }

    public void setPositionDegrees(double position) {
        pos = degreesToTicks(position);
    }

    public double getCurrentPositionDegrees() {
        if (hardware) {
            return ticksToDegrees(arm.getCurrentPosition());
        } else {
            return getTargetPositionDegrees();
        }
    }

    public int getCurrentPositionTicks() {
        if (hardware) {
            return arm.getCurrentPosition();
        } else {
            return getTargetPositionTicks();
        }
    }

    public double getTargetPositionDegrees() {
        return ticksToDegrees(pos);
    }

    public int getTargetPositionTicks() {
        return pos;
    }

    public void run() {
        if (!hardware) {
            return;
        }
        arm.setMode(runMode);
        if (runMode == DcMotor.RunMode.RUN_TO_POSITION) {
            arm.setTargetPosition(pos);
        }
        ((DcMotorEx) arm).setVelocity(velocity); // 2500
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void setVelocity(int vel) {
        velocity = vel;
        update();
    }

    public void relax() {
        if (!hardware) {
            return;
        }
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        arm.setPower(0);
    }
    public void update() {
        if (relaxed) {
            relax();
        }
        else {
            run();
        }
    }
    public boolean isRelaxed() {
        return relaxed;
    }
    public void setRelaxed(boolean relaxed) {
        this.relaxed = relaxed;
    }
    public void changeState() {
        relaxed ^= true;
    }
}
