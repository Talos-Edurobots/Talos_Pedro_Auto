package pedroPathing.examples;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import pedroPathing.actuators.*;
import pedroPathing.gamepad.GamepadButtonHandler;

@TeleOp(name = "Actuators Tester", group = "Test")
public class ActuatorsTester extends LinearOpMode {
    boolean hardware = false;
    final String ARM_CONFIGURATION    = "dc_arm";
    final String VIPER_CONFIGURATION  = "viper_motor";
    final String INTAKE_CONFIGURATION = "intake_servo";
    final String WRIST_CONFIGURATION  = "wrist_servo";
    final String OTOS_CONFIGURATION   = "otos";
    final double ARM_TICKS_PER_DEGREE = (28 // number of encoder ticks per rotation of the bare motor
            * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
            * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
            * 1/360.0);

    SparkFunOTOS otos;
    SparkFunOTOS.Pose2D otosPos;
    Actuators currentActuator;
    Arm arm;
    GobildaViper viper;
    Servos servos;
    int viperPosition = 0;
    double armPosition = 0;
    boolean ArmSetPositionMode = true; // true: sets position, false: gets position
    boolean ViperSetPositionMode = true; // true: sets position, false: gets position
    boolean showInTicks = false; // true: show ticks, false: show degrees/mm
    double loopTime, oldTime, cycleTime;
    GamepadButtonHandler myGamepad = new GamepadButtonHandler();

    @Override
    public void runOpMode() {
        arm    = new Arm(ARM_CONFIGURATION, hardwareMap, hardware);
        viper  = new GobildaViper(VIPER_CONFIGURATION, hardwareMap, DcMotor.Direction.REVERSE, hardware);
        servos = new Servos(INTAKE_CONFIGURATION, WRIST_CONFIGURATION, hardwareMap, hardware);
        if (hardware) {
            otos = hardwareMap.get(SparkFunOTOS.class, "otos");
            configureOtos();
        }

        currentActuator = Actuators.ARM;
        telemetry.addLine("Robot ready.");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            sleep(10);

            if (hardware) {
                otosPos = otos.getPosition();
            }

            switch (currentActuator) {
                case ARM:
                    armPosition -= (gamepad1.left_stick_y * 1000 * cycleTime);
                    armPosition -= (gamepad1.right_stick_y * 100 * cycleTime);
                    if (armPosition < 0) {
                        armPosition = 0;
                        gamepad1.rumbleBlips(3);
                    }
                    if (myGamepad.a.justPressed(gamepad1.a)) {
                        arm.changeState();
                    }
                    break;
                case VIPER:
                    viperPosition -= (int) (gamepad1.left_stick_y * 1000 * cycleTime);
                    viperPosition -= (int) (gamepad1.right_stick_y * 100 * cycleTime);
                    if (viperPosition < 0) {
                        viperPosition = 0;
                        gamepad1.rumble(500);
                        gamepad1.rumble(1000);
                    }
                    if (myGamepad.a.justPressed(gamepad1.a)) {
                        viper.changeState();
                    }
                    break;
                case INTAKE:
                    servos.setIntakePosition(servos.getIntakePosition() + gamepad1.left_stick_y * cycleTime);
                    servos.setIntakePosition(servos.getIntakePosition() + gamepad1.right_stick_y * cycleTime * .1f);
                case WRIST:
                    servos.setWristPosition(servos.getWristPosition() + gamepad1.left_stick_y * cycleTime);
                    servos.setWristPosition(servos.getWristPosition() + gamepad1.right_stick_y * cycleTime * .1f);
            }

            arm.setPositionTicks((int) armPosition);
            viper.setPositionTicks(viperPosition);

            arm.update();
            viper.update();
            if (myGamepad.dpad_up.justPressed(gamepad1.dpad_up)) {
                previousActuator();
            }
            if (myGamepad.dpad_down.justPressed(gamepad1.dpad_down)) {
                nextActuator();
            }
            if (myGamepad.y.justPressed(gamepad1.y)) {
                showInTicks ^= true;
            }

            myGamepad.update(gamepad1);
            loopTime = getRuntime();
            cycleTime = loopTime - oldTime;
            oldTime = loopTime;
            telemetry.addLine("*** keybindings ***");
            telemetry.addLine("dpad up: previous actuator");
            telemetry.addLine("dpad down: next actuator");
            telemetry.addLine("y: toggle show in other unit (e.g. ticks/degrees)");
            telemetry.addLine("a: change mode (e.g. set position/get position)");
            telemetry.addLine("\n**********");

            telemetry.addLine((currentActuator==Actuators.ARM ? "--- ":"") + "Arm Position:\t" + (int)(showInTicks ? arm.getCurrentPositionTicks():arm.getCurrentPositionDegrees()) + (showInTicks ? " ticks":" degrees") + ", \t" + "floating: " + arm.isRelaxed());
            telemetry.addLine(((currentActuator==Actuators.VIPER ? "--- ":"") + "Viper Position:\t") + (int)(showInTicks ? viper.getCurrentPositionTicks():viper.getCurrentPositionMm()) + (showInTicks ? " ticks":" mm") + ", \t" + "floating: " + viper.isRelaxed());
            telemetry.addLine((currentActuator==Actuators.INTAKE ? "--- ":"") + "Intake Position:	" + servos.getIntakePosition());
            telemetry.addLine((currentActuator==Actuators.WRIST ? "--- ":"") + "Wrist Position:	" + servos.getWristPosition());
            telemetry.addLine((currentActuator==Actuators.OTOS ? "--- ":"") + (hardware ? "OTOS Position:\t" + otosPos.toString():"Disabled OTOS"));
            telemetry.addLine();
            telemetry.addData("cycle time", cycleTime);
            telemetry.addData("running time", getRuntime());
            telemetry.addData("is rumbling", gamepad1.isRumbling());
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

    void nextActuator() {
        Actuators[] actuators = Actuators.values();
        int index = currentActuator.ordinal();
        if (index < actuators.length -1) {
            currentActuator = actuators[index + 1];
        }
        else {
            currentActuator = actuators[0];
        }
    }

    void previousActuator() {
        Actuators[] actuators = Actuators.values();
        int index = currentActuator.ordinal();
        if (index > 0) {
            currentActuator = actuators[index - 1];
        }
        else {
            currentActuator = actuators[actuators.length-1];
        }
    }

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
