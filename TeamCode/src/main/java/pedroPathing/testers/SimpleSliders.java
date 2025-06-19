package pedroPathing.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "testttt")
public class SimpleSliders extends LinearOpMode {
    DcMotor leftMisumi, rightMisumi;
    int r, l;
    @Override
    public void runOpMode() {
        leftMisumi = hardwareMap.dcMotor.get("left_misumi");
        rightMisumi = hardwareMap.dcMotor.get("right_misumi");
        leftMisumi.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMisumi.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMisumi.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMisumi.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMisumi.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMisumi.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMisumi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMisumi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                r += 100;
                l += 100;
            }
            else if (gamepad1.b) {
                r -= 100;
                l -= 100;
            }
            r = Math.min(r, 1450);
            l = Math.min(l, 1450);
            rightMisumi.setTargetPosition(r);
            leftMisumi.setTargetPosition(l);
            rightMisumi.setPower(1);
            leftMisumi.setPower(1);

            telemetry.addData("left current", ((DcMotorEx) leftMisumi).getCurrent(CurrentUnit.AMPS));
            telemetry.addData("right current", ((DcMotorEx) leftMisumi).getCurrent(CurrentUnit.AMPS));
            telemetry.addData("left pos", leftMisumi.getCurrentPosition());
            telemetry.addData("right pos", rightMisumi.getCurrentPosition());
            telemetry.update();
        }
    }
}
