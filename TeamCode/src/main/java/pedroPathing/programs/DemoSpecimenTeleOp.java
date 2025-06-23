package pedroPathing.programs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import pedroPathing.actuators.Arm;
import pedroPathing.actuators.Servos;
import pedroPathing.actuators.Slider;
import pedroPathing.actuators.Strafer;

public class DemoSpecimenTeleOp extends LinearOpMode {
    Arm arm;
    Slider viper;
    Servos servos;
    Strafer strafer;
    boolean hardware = true;

    @Override
    public void runOpMode() throws InterruptedException {
        arm = new Arm("dc_arm", hardwareMap, hardware);
        viper = new Slider("viper_motor", hardwareMap, DcMotor.Direction.REVERSE, hardware);
        servos = new Servos("intake_servo", "wrist_servo", hardwareMap, hardware);
        strafer = new Strafer(hardwareMap);
    }
}
