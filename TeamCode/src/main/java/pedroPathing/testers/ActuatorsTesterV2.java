package pedroPathing.testers;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import pedroPathing.actuators.*;
import pedroPathing.gamepad.GamepadButtonHandler;

@TeleOp(name = "Actuators Tester v3", group = "Test")
public class ActuatorsTesterV2 extends LinearOpMode {
    boolean hardware = true;
    final String ARM_CONFIGURATION    = "dc_arm";
    final String VIPER_CONFIGURATION  = "viper_motor";
    final String INTAKE_CONFIGURATION = "intake_servo";
    final String WRIST_CONFIGURATION  = "wrist_servo";
    final String OTOS_CONFIGURATION   = "otos";


    SparkFunOTOS otos;
    SparkFunOTOS.Pose2D otosPos;
    Actuators currentActuator;
    Arm arm;
    GobildaViper viper;
    Servos servos;
    int viperPosition = 0;
    int armPosition = 0;
    boolean ArmSetPositionMode = true; // true: sets position, false: gets position
    boolean ViperSetPositionMode = true; // true: sets position, false: gets position
    boolean showInTicks = false; // true: show ticks, false: show degrees/mm
    double loopTime, oldTime, cycleTime;
    final double DEFAULT_SCALAR = .5;
    final double LOWER_SCALAR = .1;
    final double HIGHER_SCALAR = 1;
    final int MAX_VIPER_SPEED = GobildaViper.mmToTicks(10);
    final int MAX_ARM_SPEED = Arm.degreesToTicks(30);

    GamepadButtonHandler myGamepad = new GamepadButtonHandler();

    @Override
    public void runOpMode() {
        arm    = new Arm(ARM_CONFIGURATION, hardwareMap, hardware);
        viper  = new GobildaViper(VIPER_CONFIGURATION, hardwareMap, DcMotor.Direction.REVERSE, hardware);
        servos = new Servos(INTAKE_CONFIGURATION, WRIST_CONFIGURATION, hardwareMap, hardware);
        if (hardware) {
            otos = hardwareMap.get(SparkFunOTOS.class, OTOS_CONFIGURATION);
            configureOtos();
        }
        servos.intakeCollect();
        servos.wristFolded();

        currentActuator = Actuators.ARM;

        telemetry.addLine("Robot ready.");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            sleep(10);

            if (hardware) {
                otosPos = otos.getPosition();
            }
            double scalar = (
                    DEFAULT_SCALAR
                        - gamepad1.left_trigger*(DEFAULT_SCALAR-LOWER_SCALAR)
                        + gamepad1.right_trigger*(HIGHER_SCALAR-DEFAULT_SCALAR)
                );

            switch (currentActuator) {
                case ARM:
                    if (myGamepad.a.justPressed(gamepad1.a)) {
                        arm.changeState();
                    }
                    armPosition = (int) (MAX_ARM_SPEED * scalar * (-gamepad1.left_stick_y));
                    break;
                case VIPER:
                    if (myGamepad.a.justPressed(gamepad1.a)) {
                        viper.changeState();
                    }
                    viperPosition = ((int) (MAX_VIPER_SPEED * scalar * (-gamepad1.left_stick_y)));
                    break;
                case INTAKE:
                    servos.setIntakePosition(servos.getIntakePosition() + gamepad1.left_stick_y * cycleTime * scalar);
                    break;
                case WRIST:
                    servos.setWristPosition(servos.getWristPosition() + gamepad1.left_stick_y * cycleTime * scalar);
                    break;
            }

            arm.setPositionTicks(armPosition);
            viper.setPositionTicks(viperPosition);
            arm.update();
            viper.update();
            if (myGamepad.left_bumper.justPressed(gamepad1.left_bumper)) {
                previousActuator();
            }
            if (myGamepad.right_bumper.justPressed(gamepad1.right_bumper)) {
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
        OTOS,
        STRAFE
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

        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 7, -90);
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
