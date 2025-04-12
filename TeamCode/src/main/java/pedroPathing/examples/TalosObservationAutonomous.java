package pedroPathing.examples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.examples.actuators.Arm;
import pedroPathing.examples.actuators.Servos;
import pedroPathing.examples.actuators.Viper;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@Autonomous(name="Red Observation Autonomous", group="Examples")
public class TalosObservationAutonomous extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    Arm    arm;
    Servos servos;
    Viper  viper;
    private final Pose startPose    = new Pose(0,  70, Math.toRadians(0));
    private final Pose scorePose = new Pose(27, 70, Math.toRadians(0));
    private final Pose pickup1Pose = new Pose(25, 30, Math.toRadians(0));
    private PathChain grabPickup1;
    private Path scorePreload;
    public void buildPaths() {
        scorePreload= new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                arm.setPositionDegrees(90);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
//                    follower.followPath(grabPickup1, true);
                    arm.setPositionDegrees(70);
                    setPathState(2);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        arm    = new Arm    ("dc_arm", hardwareMap, telemetry);
        viper  = new Viper  ("viper_motor", hardwareMap, telemetry);
        servos = new Servos ("intake_servo", "wrist_servo", hardwareMap);
        servos.intakeCollect();
        servos.wristHorizontal();
        arm.setPositionDegrees(10);
        arm.runArm(false);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        arm.runArm(false);


        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("follower busy", follower.isBusy());
        telemetry.update();
    }
}
