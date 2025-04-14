package pedroPathing.examples.actuators;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Viper {
    public DcMotor viper;
    ElapsedTime timer = new ElapsedTime();
    public Telemetry tel;
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
    int viperMmToTicks(double mm) {
        return (int) (mm * VIPER_TICKS_PER_MM);
    }
    double viperTicksToMm(int ticks) {
        return (ticks / VIPER_TICKS_PER_MM);
    }

    public Viper(OpMode opMode, String name, HardwareMap hwmap, Telemetry tel) {
        this.tel = tel;
        this.opMode = opMode;
        viper = hwmap.get(DcMotor.class, name); //the arm motor
        viper.setDirection(DcMotor.Direction.REVERSE);
        viper.setTargetPosition(0);
        viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ((DcMotorEx) viper).setCurrentAlert(5, CurrentUnit.AMPS);
    }
    public void setPositionMm(double position) {
        pos = viperMmToTicks(position);
//        tel.addData("input", position);
//        tel.addData("output", pos);
    }
    public void setPositionTicks(int pos) {
        this.pos = pos;
    }
    public double getPositionMm() {
        return viperTicksToMm(pos);
    }
    public void calibrateViper() {
        /*
         * This is a solution to a problem that our robot had. Sometimes during gameplay the belt of
         * the viper slide didn't made contact with the gear and consequently the belt skipped some
         * teeth in the gear. This means that when the viper slide motor runs to position that the
         * viper slide is collapsed, the viper slide motor will stall. This piece of code runs the motor
         * in order to get stalled for 1 second. Then I reset the motor and I set the current position to 0
         */
        timer.reset();
        viper.setTargetPosition(viperMmToTicks(-500));
        ((DcMotorEx) viper).setVelocity(3000); //velocity of the viper slide
        viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double i;
        do {
            i = timer.milliseconds();
            tel.addData("VIPER CALIBRATE TIME: ", i);
            ((DcMotorEx) viper).setVelocity(3000); //velocity of the viper slide
            viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        while (i < 1000);
        viper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setPositionTicks(0);
        runViper(false);
    }
    public void runViper(boolean sequentially) {
        viper.setTargetPosition(pos);
        viper.setPower(.5);
        viper.setMode(DcMotor.RunMode.RUN_TO_POSITION); // we finally run the arm motor
        if (!sequentially) {
            return;
        }
        while (viper.isBusy()) {
            continue;
        }
    }
}
