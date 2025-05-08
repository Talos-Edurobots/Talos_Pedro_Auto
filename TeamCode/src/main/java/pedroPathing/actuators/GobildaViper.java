package pedroPathing.actuators;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class GobildaViper {
    public boolean hardware;
    private boolean relaxed;
    public DcMotor viper;
    ElapsedTime timer = new ElapsedTime();
//    public Telemetry tel;
    private OpMode opMode;
    private int pos; // in ticks
    private int viperCalibrationAmount = 100;
    private static double VIPER_TICKS_PER_MM = (
            (
                    (
                            537.7 * 5.8
                    ) // total ticks
                            / 696
            ) // viper slide unfolded length
    );
    // to achieve its target 0mm positionn. This has the result the motor to heat up and get stalled and get destroyed. However the viper motor always achieves the target for
    //100mm position and thus doesn't get streesed.
    static int viperMmToTicks(double mm) {
        return (int) (mm * VIPER_TICKS_PER_MM);
    }
    static double viperTicksToMm(int ticks) {
        return (ticks / VIPER_TICKS_PER_MM);
    }
    void init(String name, HardwareMap hwmap) {
        viper = hwmap.get(DcMotor.class, name); //the arm motor
        viper.setDirection(DcMotor.Direction.REVERSE);
        viper.setTargetPosition(0);
        viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ((DcMotorEx) viper).setCurrentAlert(5, CurrentUnit.AMPS);
    }

    public GobildaViper(String name, HardwareMap hwmap) {
        init(name, hwmap);
    }
    public GobildaViper(String name, HardwareMap hwmap, boolean hardware) {
        if (hardware) {
            init(name, hwmap);
        }
        else {
            hardware = false;
        }
    }
    public void setPositionMm(double position) {
        pos = viperMmToTicks(position);
    }
    public void setPositionTicks(int pos) {
        this.pos = pos;
    }
    public double getCurrentPositionMm() {
        if (hardware) {
            return viperTicksToMm(viper.getCurrentPosition());
        }
        else {
            return getTargetPositionMm();
        }
    }
    public int getCurrentPositionTicks() {
        if (hardware) {
            return viper.getCurrentPosition();
        }
        else {
            return getTargetPositionTicks();
        }
    }
    public double getTargetPositionMm() {
        return viperTicksToMm(pos);
    }
    public int getTargetPositionTicks() {
        return pos;
    }
    public void calibrateViper() {
        /*
         * This is a solution to a problem that our robot had. Sometimes during gameplay the belt of
         * the viper slide didn't made contact with the gear and consequently the belt skipped some
         * teeth in the gear. This means that when the viper slide motor runs to position that the
         * viper slide is collapsed, the viper slide motor will stall. This piece of code runs the motor
         * in order to get stalled for 1 second. Then I reset the motor and I set the current position to 0
         */
        relaxed = false;
        Thread thread = new Thread() {
            @Override
            public void run() {
                timer.reset();
                viper.setTargetPosition(viperMmToTicks(-500));
                ((DcMotorEx) viper).setVelocity(3000); //velocity of the viper slide
                viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                double i;
                do {
                    i = timer.milliseconds();
//            tel.addData("VIPER CALIBRATE TIME: ", i);
//            tel.update();
                    ((DcMotorEx) viper).setVelocity(3000); //velocity of the viper slide
                    viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                while (i < 1000);
                viper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                setPositionTicks(50);
                GobildaViper.this.run(false);
            }
        };
        thread.start();
    }
    public void run(boolean usingPower) {

        relaxed = false;
        viper.setTargetPosition(pos);
        if (usingPower) {
            viper.setPower(.5);
        }
        else {
            ((DcMotorEx) viper).setVelocity(3000);
        }
        viper.setMode(DcMotor.RunMode.RUN_TO_POSITION); // we finally run the arm motor
    }
    public void relax() {
        relaxed = true;
        viper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        viper.setPower(0);
    }
    public void update() {
        if (!hardware) {
            return;
        }
        if (relaxed) {
            relax();
            return;
        }
        run(false);
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
