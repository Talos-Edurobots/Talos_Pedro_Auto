package pedroPathing.programs;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import pedroPathing.PPConstants.FConstants;
import pedroPathing.PPConstants.LConstants;
import pedroPathing.actuators.Arm;
import pedroPathing.actuators.Slider;
import pedroPathing.actuators.Servos;

/** This is the program for the autonomous period on the Observation Zone for this year.
 * We start at the red side of the field, and we score the preloaded specimen
 * then we push the three samples to the observation zone and score another specimen.
 * */

/**
 *
*/
@Autonomous(name="Red Observation Autonomous Reverse", group="Examples")
@Disabled
public class TalosObservationAutonomousReverse extends OpMode {
    /** We create a Follower object from PedroPathing*/
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private ElapsedTime timer;
    private int pathState;
    Arm    arm;
    Servos servos;
    Slider viper;
    final double ARM_GRAB_SPECIMEN_DEGREES = 22;
    final double ARM_ATTACH_TO_BAR_DEGREES = 77;
    final double ARM_SCORE_DEGREES = 60;
    // the starting pose of the robot
    private final Pose startPose    = new Pose(10,  66.5, Math.toRadians(180)); // reverse
    // the pose of the robot when it is going to score the preloaded specimen
    private final Pose scorePreloadPose = new Pose(38, 66.5, Math.toRadians(180)); // reverse
    // the control point of the bezier curve that goes from the score preloaded pose to the samples pose
//    private final Pose samplesControlPoint1 = new Pose(14.5, 36.5, Math.toRadians(0));
    private final Pose samplesControlPoint1 = new Pose(6, 20.7, Math.toRadians(0));
    private final Pose samplesControlPoint2 = new Pose(62.7, 47, Math.toRadians(0));
    // the pose of the robot when it is next to the samples and the submersible
//    private final Pose samplesPose = new Pose(55, 34.5, Math.toRadians(0));
    private final Pose samplesPose = new Pose(60, 24, Math.toRadians(0));
    // the pose of the robot when the back side of it is next to the first sample in order to push it in the observation zone
    private final Pose sample1Pose = new Pose(60, 33, Math.toRadians(0));
    // the pose of the robot when it pushes the first sample in the observation zone
    private final Pose sample1ControlPose = new Pose(60, 25, Math.toRadians(0));
//    private final Pose attach1Pose = new Pose(21, 31.5, Math.toRadians(0));
    private final Pose attach1Pose = new Pose(30, 33, Math.toRadians(0));
    // the pose of the robot when the back side of it is next to the second sample in order to push it in the observation zone
    private final Pose sample2Pose = new Pose(60, 22, Math.toRadians(0));
    // the pose of the robot when it pushes the second sample in the observation zone
//    private final Pose sample2ControlPose = new Pose(100, 14.5, Math.toRadians(0));
    private final Pose sample2ControlPose = new Pose(60, 30, Math.toRadians(0));
//    private final Pose attach2Pose = new Pose(21, 16.5, Math.toRadians(0));
    private final Pose attach2Pose = new Pose(21, 22, Math.toRadians(0));
    // the pose of the robot when the back side of it is next to the third sample in order to push it in the observation zone
    private final Pose sample3Pose = new Pose(60, 13, Math.toRadians(0));
    // the pose of the robot when it pushes the third sample in the observation zone
    private final Pose sample3ControlPose = new Pose(60, 20, Math.toRadians(0));
    private final Pose sample3Control1Pose = new Pose(94.5, 24.9, Math.toRadians(0));
    private final Pose sample3Control2Pose = new Pose(71.2, 0.6, Math.toRadians(0));
    private final Pose attach3Pose = new Pose(30, 13, Math.toRadians(0));
    // the control point of the bezier curve that goes from the attach3Pose to the preGrabPose
    private final Pose preGrabControlPoint1 = new Pose(40, 15, Math.toRadians(0)); // x: 50
    private final Pose preGrabControlPoint2 = new Pose(40, 35, Math.toRadians(0)); // x: 50
    // the pose of the robot when it is a bit behind the grabSpecimenPose
    private final Pose preGrabPose = new Pose(22, 26.5, Math.toRadians(180));
    // the pose of the robot when it is going to grab the specimen
    private final Pose grabSpecimenPose = new Pose(23, 28, Math.toRadians(180));
    // the control point of the bezier curve that goes from the grabSpecimenPose to the scoreFirstPose
    private final Pose submersibleToObservationControlPoint = new Pose(39, 26.5, Math.toRadians(0)); // observationToSubmersibleControlPoint
    // the pose of the robot when it is going to score the first specimen
    private final Pose observationToSubmersibleControlPoint = new Pose(20, 67.5, Math.toRadians(0));
    private final Pose scoreFirstPose = new Pose(37, 67.5, Math.toRadians(0));
    private final Pose grabSecondPose = new Pose(20, 26.5, Math.toRadians(170));
    private final Pose scoreSecondPose = new Pose(37, 68.5, Math.toRadians(350));
    private final Pose grabThirdPose = new Pose(17.5, 26.5, Math.toRadians(160));
    private final Pose scoreThirdPose = new Pose(45, 69.5, Math.toRadians(340));
    private final Pose parkPose = new Pose(10, 40, Math.toRadians(340));
    private Thread armGrabWithDelay = new Thread(new Runnable() {
        @Override
        public void run() {
            try {
                Thread.sleep(1000);
                arm.setPositionDegrees(ARM_GRAB_SPECIMEN_DEGREES);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    });
    private Path scorePreload;
    private Path samples;
    private PathChain attachSamples;
    private Path preGrabFirst;
    private Path grabFirst;
    private Path scoreFirst;
    private Path grabSecond;
    private Path scoreSecond;
    private Path grabThird;
    private Path scoreThird;
    private Path park;
    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePreloadPose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePreloadPose.getHeading());
        samples = new Path(new BezierCurve(new Point(scorePreloadPose), new Point(samplesControlPoint1), new Point(samplesControlPoint2), new Point(sample1Pose)));
        samples.setLinearHeadingInterpolation(scorePreloadPose.getHeading(), sample1Pose.getHeading());
        attachSamples = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample1Pose), new Point(attach1Pose)))
                .setLinearHeadingInterpolation(sample1Pose.getHeading(), attach1Pose.getHeading())

                .addPath(new BezierCurve(new Point(attach1Pose), new Point(sample2ControlPose), new Point(sample2Pose)))
                .setLinearHeadingInterpolation(attach1Pose.getHeading(), sample2Pose.getHeading())

                .addPath(new BezierLine(new Point(sample2Pose), new Point(attach2Pose)))
                .setLinearHeadingInterpolation(sample2Pose.getHeading(), attach2Pose.getHeading())

//                .addPath(new BezierCurve(new Point(attach2Pose), new Point(sample3ControlPose), new Point(sample3Pose)))
//                .setLinearHeadingInterpolation(attach2Pose.getHeading(), attach3Pose.getHeading())

//                .addPath(new BezierLine(new Point(sample3Pose), new Point(attach3Pose)))
//                .setLinearHeadingInterpolation(sample3Pose.getHeading(), attach3Pose.getHeading(
//                ))
//                .setPathEndTimeoutConstraint(0)

                .build();

//        preGrabFirst = new Path(new BezierCurve(new Point(attach2Pose),  new Point(preGrabPose)));
//        preGrabFirst.setLinearHeadingInterpolation(attach3Pose.getHeading(), preGrabPose.getHeading());
//        preGrabFirst.setTangentHeadingInterpolation();

        grabFirst = new Path(new BezierCurve(new Point(attach2Pose),new Point(preGrabControlPoint1), new Point(preGrabControlPoint2), new Point(grabSpecimenPose)));
        grabFirst.setLinearHeadingInterpolation(attach2Pose.getHeading(), grabSpecimenPose.getHeading());

        scoreFirst = new Path(new BezierCurve(new Point(grabSpecimenPose), new Point(observationToSubmersibleControlPoint), new Point(scoreFirstPose)));
//        scoreFirst.setTangentHeadingInterpolation();
        scoreFirst.setLinearHeadingInterpolation(grabSpecimenPose.getHeading(), scoreFirstPose.getHeading());

        grabSecond = new Path(new BezierCurve(new Point(scoreFirstPose), new Point(submersibleToObservationControlPoint), new Point(grabSecondPose)));
        grabSecond.setLinearHeadingInterpolation(scoreFirstPose.getHeading(), grabSecondPose.getHeading());

        scoreSecond = new Path(new BezierCurve(new Point(grabSpecimenPose), new Point(observationToSubmersibleControlPoint), new Point(scoreSecondPose)));
        scoreSecond.setLinearHeadingInterpolation(grabSpecimenPose.getHeading(), scoreSecondPose.getHeading());

        grabThird = new Path(new BezierCurve(new Point(scoreSecondPose), new Point(submersibleToObservationControlPoint), new Point(grabThirdPose)));
        grabThird.setLinearHeadingInterpolation(scoreSecondPose.getHeading(), grabThirdPose.getHeading());

        scoreThird = new Path(new BezierCurve(new Point(grabSpecimenPose), new Point(observationToSubmersibleControlPoint), new Point(scoreThirdPose)));
        scoreThird.setLinearHeadingInterpolation(grabSpecimenPose.getHeading(), scoreThirdPose.getHeading());

        park = new Path(new BezierLine(new Point(scoreThirdPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scoreThirdPose.getHeading(), parkPose.getHeading());
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                arm.setPositionDegrees(ARM_ATTACH_TO_BAR_DEGREES);
                setPathState(1);
                break;
            case 1:
//                if (!(follower.isBusy() || arm.arm.isBusy())) { //  || pathTimer.getElapsedTime() < 1000
//                    arm.setPositionDegrees(ARM_SCORE_DEGREES);
//                    setPathState(2);
//                }
//                break;
            case 2:
                if (!(arm.arm.isBusy() || pathTimer.getElapsedTime() < 100)) {
                    servos.intakeOpen();
                    follower.followPath(samples, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!(follower.isBusy())) {
                    arm.setPositionDegrees(ARM_GRAB_SPECIMEN_DEGREES);
                    viper.setPositionMm(100);
                    follower.followPath(attachSamples, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!(follower.isBusy())) { //  || pathTimer.getElapsedTime() < 1000
//                    arm.setPositionDegrees(ARM_GRAB_SPECIMEN_DEGREES);
//                    servos.intakeCollect();
//                    follower.followPath(preGrabFirst, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!(follower.isBusy()  || arm.arm.isBusy() /*|| pathTimer.getElapsedTime() < 100*/)) { // 3000
                    viper.setPositionTicks(50);
                    servos.intakeOpen();
                    follower.followPath(grabFirst, true);
//                    servos.intakeCollect();
                    setPathState(6);
                }
                break;
            case 6:
                if (!(follower.isBusy())) { //  || pathTimer.getElapsedTime() < 1000
                    servos.intakeCollect();
                    setPathState(7);
                }
                break;
            case 7:
                if (!(pathTimer.getElapsedTime() < 700)) {
//                    arm.setPositionDegrees(50);
                    setPathState(8);
                }
                break;
            case 8:
                if (!(arm.arm.isBusy())) { //  || pathTimer.getElapsedTime() < 1000
                    arm.setPositionDegrees(ARM_ATTACH_TO_BAR_DEGREES);
                    follower.followPath(scoreFirst, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (!(follower.isBusy() || arm.arm.isBusy())) { //  || pathTimer.getElapsedTime() < 1000
                    arm.setPositionDegrees(ARM_SCORE_DEGREES);
                    setPathState(10);
                }
                break;
            case 10:
                if (!(arm.arm.isBusy())) { //  || pathTimer.getElapsedTime() < 1000
                    servos.intakeOpen();
                    setPathState(11);
                }
                break;
            case 11:
                if (!(arm.arm.isBusy() || pathTimer.getElapsedTime() < 100)) {
                    follower.followPath(grabSecond);
//                    servos.intakeCollect();
//                    arm.setPositionDegrees(ARM_GRAB_SPECIMEN_DEGREES);
                    armGrabWithDelay.start();
                    setPathState(12);
                }
                break;
            case 12:
                if (!(follower.isBusy() || pathTimer.getElapsedTime() < 500)) {
//                    follower.followPath(grabFirst);
                    setPathState(13);
                }
                break;
            case 13:
                if (!(follower.isBusy())) { //  || pathTimer.getElapsedTime() < 1000
                    servos.intakeCollect();
                    setPathState(14);
                }
                break;
            case 14:
                if (!(pathTimer.getElapsedTime() < 700)) {
//                    servos.intakeCollect();
                    setPathState(15);
                }
                break;
            case 15:
                if (!(pathTimer.getElapsedTime() < 200)) {
                    arm.setPositionDegrees(ARM_ATTACH_TO_BAR_DEGREES);
                    follower.followPath(scoreSecond);
                    setPathState(16);
                }
                break;
            case 16:
                if (!(follower.isBusy())) { // pathTimer.getElapsedTime() < 1000 ||
                    arm.setPositionDegrees(ARM_SCORE_DEGREES);
                    setPathState(17);
                }
                break;
            case 17:
                if (!(arm.arm.isBusy())) { // pathTimer.getElapsedTime() < 1000 ||
                    servos.intakeOpen();
                    setPathState(18);
                }
                break;
            case 18:
                if (!(pathTimer.getElapsedTime() < 100)) {
                    follower.followPath(grabThird);
//                    arm.setPositionDegrees(ARM_GRAB_SPECIMEN_DEGREES);
                    armGrabWithDelay.start();
                    setPathState(19);
                }
                break;
            case 19:
                if (!(follower.isBusy() || pathTimer.getElapsedTime() < 500)) {
                    servos.intakeCollect();
                    setPathState(20);
                }
                break;
            case 20:
                if (!(pathTimer.getElapsedTime() < 800)) { //  ||
                    arm.setPositionDegrees(ARM_ATTACH_TO_BAR_DEGREES);
                    follower.followPath(scoreThird);
                    setPathState(21);
                }
                break;
            case 21:
                if (!(follower.isBusy())) { //  || pathTimer.getElapsedTime() < 1000
                    arm.setPositionDegrees(ARM_SCORE_DEGREES);
                    setPathState(22);
                }
                break;
            case 22:
                if (!(arm.arm.isBusy())) { //  || pathTimer.getElapsedTime() < 1000
                    servos.intakeOpen();
                    setPathState(23);
                }
                break;
            case 23:
                if (!(pathTimer.getElapsedTime() < 100)) {
                    follower.followPath(park);
                    setPathState(24);
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
        arm    = new Arm    ("dc_arm", hardwareMap);
        viper  = new Slider("viper_motor", hardwareMap);
        servos = new Servos ("intake_servo", "wrist_servo", hardwareMap);
        telemetry.addData("Status", "init");
        telemetry.update();
        telemetry.addData("Status", "after_init");
        telemetry.update();
        servos.intakeCollect();
        servos.wristGrabSample();
        arm.setPositionDegrees(10);
        arm.run();
        viper.calibrate();

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

    }
    @Override
    public void init_loop(){
        telemetry.addData("Status", "init_loop");
        telemetry.update();
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
        arm.run();
        viper.run();


        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("state time", pathTimer.getElapsedTime());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("follower busy", follower.isBusy());
        telemetry.addData("arm busy", follower.isBusy());
        telemetry.addData("elapsed time < 1000 ", pathTimer.getElapsedTime() < 1000);
        telemetry.addData("viper current", ((DcMotorEx) viper.viper).getCurrent(CurrentUnit.AMPS));
        telemetry.addData("viper pos",  viper.getCurrentPositionMm());
        telemetry.addData("arm degrees", arm.getCurrentPositionDegrees());
        telemetry.addData("follower error", follower.driveError);
        telemetry.update();
    }
    @Override
    public void stop() {
        viper.setPositionTicks(0);
        arm.setPositionDegrees(0);
        viper.run();
        arm.run();
    }
}
