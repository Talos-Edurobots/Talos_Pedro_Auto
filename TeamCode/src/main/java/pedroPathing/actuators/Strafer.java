package pedroPathing.actuators;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Strafer {
    public DcMotor frontLeft, frontRight, backLeft, backRight;
    public Strafer(HardwareMap hwmap) {
        frontLeft = hwmap.dcMotor.get("front_left");
        frontRight = hwmap.dcMotor.get("front_right");
        backLeft = hwmap.dcMotor.get("back_left");
        backRight = hwmap.dcMotor.get("back_right");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void fieldCentric(double x, double y, double rx, double botHeading, double speed) {

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX *= 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        // we multiply denominator variable by a variable named "straferSpeedFactor" with a value greater than 1
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = speed*((rotY + rotX + rx) / denominator);
        double backLeftPower = speed*((rotY - rotX + rx) / denominator);
        double frontRightPower = speed*((rotY - rotX - rx) / denominator);
        double backRightPower = speed*(rotY + rotX - rx) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }
}
