package pedroPathing.examples.actuators;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Viper {
    public DcMotor viper;
    public Telemetry tel;
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

    public Viper(String name, HardwareMap hwmap, Telemetry tel) {
        this.tel = tel;
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
        tel.addData("input", position);
        tel.addData("output", pos);
    }
    public void setPositionTicks(int pos) {
        this.pos = pos;
    }
    public double getPositionMm() {
        return viperTicksToMm(pos);
    }
    public void runViper(boolean sequentially) {
        viper.setTargetPosition(pos);
        viper.setPower(.2);
        viper.setMode(DcMotor.RunMode.RUN_TO_POSITION); // we finally run the arm motor
        if (!sequentially) {
            return;
        }
        while (viper.isBusy()) {
            continue;
        }
    }
}
