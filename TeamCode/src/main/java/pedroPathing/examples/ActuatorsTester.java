package pedroPathing.examples;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ActuatorsTester extends LinearOpMode {
    final String ARM_CONFIGURATION    = "dc_arm";
    final String VIPER_CONFIGURATION  = "viper_motor";
    final String INTAKE_CONFIGURATION = "intake_servo";
    final String WRIST_CONFIGURATION  = "wrist_servo";
    final String OTOS_CONFIGURATION   = "otos";
    final double ARM_TICKS_PER_DEGREE = (28 // number of encoder ticks per rotation of the bare motor
            * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
            * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
            * 1/360.0);
    DcMotor armMotor, viperMotor;
    Servo intakeServo, wristServo;
    SparkFunOTOS otos;
    SparkFunOTOS.Pose2D otosPos;
    Actuators chosenActuator;
    boolean setPositionMode = true; // true: sets position, false: gets position

    @Override
    public void runOpMode() throws InterruptedException {
        armMotor    = hardwareMap.dcMotor.get(ARM_CONFIGURATION);
        viperMotor  = hardwareMap.dcMotor.get(VIPER_CONFIGURATION);
        intakeServo = hardwareMap.servo.  get(INTAKE_CONFIGURATION);
        wristServo  = hardwareMap.servo.  get(WRIST_CONFIGURATION);
        otos        = hardwareMap.        get(SparkFunOTOS.class, OTOS_CONFIGURATION);

        armMotor.setTargetPosition(0);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        viperMotor.setTargetPosition(0);
        viperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        viperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intakeServo.setPosition(0);
        wristServo.setPosition(0);

        configureOtos();

        chosenActuator = Actuators.ARM;
        telemetry.addLine("Robot ready.");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            otosPos = otos.getPosition();
            armMotor.setPower(.5);
            viperMotor.setPower(.5);

            switch (chosenActuator) {
                case ARM:
                    armMotor.setTargetPosition((int) Math.max(gamepad1.left_stick_y * 1000, gamepad1.right_stick_y * 100));
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    break;
                case VIPER:
                    viperMotor.setTargetPosition((int) Math.max(gamepad1.left_stick_y * 1000, gamepad1.right_stick_y * 100));
                    viperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    break;
            }

            telemetry.addLine(chosenActuator==Actuators.OTOS ? "--- ":"" + "OTOS Position:\t" + otosPos.toString());
            telemetry.update();
        }
    }
    enum Actuators {
        ARM,
        VIPER,
        INTAKE,
        WRIST,
        OTOS
    };
    void configureOtos() {
//        telemetry.addLine("Configuring OTOS...");
//        telemetry.update();


        otos.setLinearUnit(DistanceUnit.CM);
        otos.setAngularUnit(AngleUnit.DEGREES);

        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setOffset(offset);

        otos.setLinearScalar(1.138); //known distance 182cm, measured distance 160cm, error 182/160 = 1.138
        otos.setAngularScalar(0.9978); // 10 rotations 3600 degrees, measured 3608, error 3600/3608 =0.9978

        otos.calibrateImu();

        otos.resetTracking();

        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        otos.getVersionInfo(hwVersion, fwVersion);

        otosPos = otos.getPosition();

//        telemetry.addLine("OTOS configured! Press start to get position data!");
//        telemetry.addLine();
//        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
//        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
//        telemetry.update();
    }
}
