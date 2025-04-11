package pedroPathing.examples;

import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.examples.intake.Arm;
import pedroPathing.examples.intake.Servos;
import pedroPathing.examples.intake.Viper;


@Autonomous(name="Red Observation Autonomous", group="Examples")
public class TalosObservationAutonomous extends OpMode{
    Arm    arm;
    Servos servos;
    Viper  viper;
    final Pose startPose    = new Pose(0,  70, Math.toRadians(270));
    final Pose scorePreload = new Pose(30, 70, Math.toRadians(270));

    @Override
    public void init() {
        arm    = new Arm    ("dc_arm", hardwareMap, telemetry);
        viper  = new Viper  ("viper_motor", hardwareMap, telemetry);
        servos = new Servos ("intake_servo", "wrist_servo", hardwareMap);
        servos.intakeCollect();
        servos.wristHorizontal();
    }

    @Override
    public void loop() {
        servos.intakeOpen();
        servos.wristVertical();
        viper.setPositionMm(100);
        arm.setPositionDegrees(80);
        arm.runArm();
        telemetry.addData("arm pos", arm.arm.getCurrentPosition());
        telemetry.addData("viper pos", viper.viper.getCurrentPosition());
        telemetry.update();
        viper.runViper();
    }
}
