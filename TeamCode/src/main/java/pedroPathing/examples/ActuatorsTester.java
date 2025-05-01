package pedroPathing.examples;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import pedroPathing.actuators.*;
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
    Actuators chosenActuator;
    Arm arm;
    Viper viper;
    Servos servos;
    int viperPosition = 0, armPosition = 0;
    boolean ArmSetPositionMode = true; // true: sets position, false: gets position
    boolean ViperSetPositionMode = true; // true: sets position, false: gets position
    boolean ShowTicks = true; // true: show ticks, false: show degrees/mm
    double loopTime, oldTime, cycleTime;

    @Override
    public void runOpMode() throws InterruptedException {
        arm    = new Arm(ARM_CONFIGURATION, hardwareMap, hardware);
        viper  = new Viper(VIPER_CONFIGURATION, hardwareMap, hardware);
//        servos = new Servos(INTAKE_CONFIGURATION, WRIST_CONFIGURATION, hardwareMap);
//        configureOtos();

        chosenActuator = Actuators.ARM;
        telemetry.addLine("Robot ready.");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
//            otosPos = otos.getPosition();


            switch (chosenActuator) {
                case ARM:
                    armPosition += (int) (gamepad1.left_stick_y * 1000 * cycleTime);
                    break;
                case VIPER:
                    viperPosition += (int) (gamepad1.left_stick_y * 1000 * cycleTime);
                    break;
            }

//            telemetry.addLine(chosenActuator==Actuators.OTOS ? "--- ":"" + "OTOS Position:\t" + otosPos.toString());
            telemetry.addLine(chosenActuator==Actuators.VIPER ? "--- ":"" + "Viper Position:\t" + (ShowTicks ? viper.getCurrentPositionTicks():viper.getCurrentPositionMm()));
            telemetry.addLine(chosenActuator==Actuators.ARM ? "--- ":"" + "Arm Position:\t" + (ShowTicks ? arm.getCurrentPositionTicks():arm.getCurrentPositionDegrees()));
            telemetry.update();

            loopTime = getRuntime();
            cycleTime = loopTime - oldTime;
            oldTime = loopTime;
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
