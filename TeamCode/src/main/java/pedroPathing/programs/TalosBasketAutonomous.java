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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import pedroPathing.PPConstants.FConstants;
import pedroPathing.PPConstants.LConstants;
import pedroPathing.actuators.Arm;
import pedroPathing.actuators.Servos;
import pedroPathing.actuators.Slider;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "Red Basket Autonomous", group = "Examples")
@Disabled
public class TalosBasketAutonomous extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    Slider viper;
    Arm arm;
    Servos servos;
    final double ARM_SCORE_DEGREES = 107;
    final double VIPER_SCORE_MM= 500;


    /** This is the variable where we store the state of our auto.
     * It is used by the autonomousPathUpdate method. */
    private int pathState;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(10, 111, Math.toRadians(0));
    private final Pose testPos = new Pose(10, 111, Math.toRadians(180));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */

    private final Pose preScorePose = new Pose(30, 120, Math.toRadians(135));
    private final Pose scorePreloadPose = new Pose(23, 119, Math.toRadians(135));
    private final Pose scorePreloadControlPose = new Pose(26, 81, Math.toRadians(135));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickup1Pose = new Pose(30, 113, Math.toRadians(0));
    private final Pose pickup1ControlPose = new Pose(10, 90, Math.toRadians(0));
    private final Pose scorePickup1ControlPose = new Pose(9, 101, Math.toRadians(0));
    private final Pose scoreGrab1Pose = new Pose(17, 117, Math.toRadians(135));
    private final Pose scorePickup2ControlPose = new Pose(30, 100, Math.toRadians(0));

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose pickup2Pose = new Pose(33, 126, Math.toRadians(0));
    private final Pose pickup2ControlPose = new Pose(10, 90, Math.toRadians(0));
    private final Pose scoreGrab2Pose = new Pose(22.5, 120, Math.toRadians(135));

    /** Highest (Third) Sample from the Spike Mark */
    private final Pose pickup3Pose = new Pose(49, 123.5, Math.toRadians(90));
    private final Pose scoreGrab3Pose = new Pose(18, 118, Math.toRadians(135));

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(65, 98, Math.toRadians(90));

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    private final Pose parkControlPose = new Pose(80, 100, Math.toRadians(90));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path park, test;
    private PathChain scorePreload, grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;
    private boolean viperMode = false;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(startPose), new Point(scorePreloadControlPose), new Point(scorePreloadPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePreloadPose.getHeading())

//                .addPath(new BezierLine(new Point(preScorePose), new Point(scorePose)))
//                .setLinearHeadingInterpolation(preScorePose.getHeading(), scorePose.getHeading())
                .build();

        test = new Path(new BezierLine(new Point(startPose), new Point(testPos)));
        test.setLinearHeadingInterpolation(startPose.getHeading(), testPos.getHeading());

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePreloadPose), new Point(pickup1ControlPose), new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(scorePreloadPose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickup1Pose), new Point(scorePickup1ControlPose), new Point(scoreGrab1Pose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePreloadPose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scoreGrab1Pose),new Point(pickup1ControlPose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(scorePreloadPose.getHeading(), pickup2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickup2Pose), new Point(scorePickup2ControlPose), new Point(scoreGrab2Pose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scoreGrab2Pose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scoreGrab2Pose), new Point(pickup1ControlPose), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(scorePreloadPose.getHeading(), pickup3Pose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickup3Pose), new Point(pickup1ControlPose), new Point(scoreGrab3Pose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scoreGrab3Pose.getHeading())
                .build();

        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        park = new Path(new BezierCurve(new Point(scoreGrab3Pose), /* Control Point */ new Point(parkControlPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scoreGrab3Pose.getHeading(), parkPose.getHeading());
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload, true);
                arm.setPositionDegrees(ARM_SCORE_DEGREES);
//                follower.followPath(test, true);
                setPathState(1);
                break;
            case 1:
                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */


                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!(/*follower.isBusy() || */ arm.arm.isBusy())) {
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                    follower.followPath(grabPickup1,true);
                    viper.setPositionMm(VIPER_SCORE_MM);
                    setPathState(2);
                }
                break;
            case 2:

                setPathState(3);

                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!viper.viper.isBusy()) {
                    servos.intakeOpen();
                    setPathState(4);
                }
                break;
            case 4:
                if(!(pathTimer.getElapsedTime() < 500)) {
                    arm.setPositionDegrees(ARM_SCORE_DEGREES+5);
                    setPathState(5);
                }
                break;
            case 5:
                if(!(arm.arm.isBusy())) {
                    viper.setPositionTicks(20);
                    follower.followPath(grabPickup1, true);
                    setPathState(6);
                }
                break;
            case 6:
                if(!(viper.getCurrentPositionTicks() > 25 || pathTimer.getElapsedTime() < 1500)) {
                    arm.setPositionDegrees(0);
                    setPathState(7);
                }
                break;
            case 7:
                setPathState(8);

                break;
            case 8:
                if(!follower.isBusy()) {
                    viper.setPositionMm(50);
                    setPathState(9);
                }
                break;
            case 9:
                if(!viper.viper.isBusy()) {
                    servos.intakeCollect();
                    setPathState(10);
                }
                break;
            case 10:
                if(!(pathTimer.getElapsedTime() < 800)) {
                    follower.followPath(scorePickup1, true);
                    arm.setPositionDegrees(ARM_SCORE_DEGREES);
                    setPathState(11);
                }
                break;
            case 11:
                setPathState(12);
                break;
            case 12:
                setPathState(13);
                break;
            case 13:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!arm.arm.isBusy()) {
                    viper.setPositionMm(VIPER_SCORE_MM);
                    setPathState(14);
                }
                break;
            case 14:
                if(!(viper.viper.isBusy() || follower.isBusy())) {
                    servos.intakeOpen();
                    setPathState(15);
                }
                break;
            case 15:
                if(!(pathTimer.getElapsedTime() < 800)) {
                    arm.setPositionDegrees(ARM_SCORE_DEGREES+5);
                    setPathState(16);
                }
                break;
            case 16:
                if(!(arm.arm.isBusy())) {
                    viper.setPositionTicks(20);
                    follower.followPath(grabPickup2, true);
                    setPathState(17);
                }
                break;
            case 17:
                if(!(viper.getCurrentPositionMm() > 100 || pathTimer.getElapsedTime() < 1500)) {
                    arm.setPositionDegrees(0);
                    setPathState(18);
                }
                break;
            case 18:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */

                setPathState(19);

                break;
            case 19:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    viper.setPositionMm(50);
                    setPathState(20);
                }
                break;
            case 20:
                if(!viper.viper.isBusy()) {
                    servos.intakeCollect();
                    setPathState(21);
                }
                break;
            case 21:
                if(!(pathTimer.getElapsedTime() < 800)) {
                    follower.followPath(scorePickup2, true);
                    arm.setPositionDegrees(ARM_SCORE_DEGREES);
                    setPathState(22);
                }
                break;
            case 22:
                setPathState(23);
                break;
            case 23:
                setPathState(24);
                break;
            case 24:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!arm.arm.isBusy()) {
                    viper.setPositionMm(VIPER_SCORE_MM);
                    setPathState(25);
                }
                break;
            case 25:
                if(!(viper.viper.isBusy() || follower.isBusy())) {
                    servos.intakeOpen();
                    setPathState(26);
                }
                break;
            case 26:
                if(!(pathTimer.getElapsedTime() < 800)) {
                    arm.setPositionDegrees(ARM_SCORE_DEGREES+5);
                    setPathState(27);
                }
                break;
            case 27:
                if(!(arm.arm.isBusy())) {
                    viper.setPositionTicks(20);
                    follower.followPath(grabPickup3, true);
                    setPathState(28);
                }
                break;
            case 28:
                if(!(viper.getCurrentPositionMm() > 100 || pathTimer.getElapsedTime() < 1500)) {
                    arm.setPositionDegrees(0);
                    setPathState(29);
                }
                break;
            case 29:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */

                setPathState(30);

                break;
            case 30:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    viper.setPositionMm(50);
                    setPathState(31);
                }
                break;
            case 31:
                if(!viper.viper.isBusy()) {
                    servos.intakeCollect();
                    setPathState(32);
                }
                break;
            case 32:
                if(!(pathTimer.getElapsedTime() < 800)) {
                    follower.followPath(scorePickup3, true);
                    arm.setPositionDegrees(ARM_SCORE_DEGREES);
                    setPathState(33);
                }
                break;
            case 33:
                setPathState(34);

                break;
            case 34:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                setPathState(35);

                break;
            case 35:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!arm.arm.isBusy()) {
                    viper.setPositionMm(VIPER_SCORE_MM);
                    setPathState(36);
                }
                break;
            case 36:
                if(!(viper.viper.isBusy() || follower.isBusy())) {
                    servos.intakeOpen();
                    setPathState(37);
                }
                break;
            case 37:
                if(!(pathTimer.getElapsedTime() < 800)) {
                    arm.setPositionDegrees(ARM_SCORE_DEGREES+5);
                    setPathState(38);
                }
                break;
            case 38:
                if(!(arm.arm.isBusy())) {
                    viper.setPositionTicks(50);
                    follower.followPath(park, true);
                    setPathState(39);
                }
                break;
            case 39:
                if(!viper.viper.isBusy()) {
                    setPathState(40);
                }
                break;
            case 40:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    setPathState(41);
                }
                break;
            case 41:
                if(!viper.viper.isBusy()) {
                    arm.setPositionDegrees(150);
                    setPathState(42);
                }
                break;
            case 42:
                if(!arm.arm.isBusy()) {
                    arm.setRelaxed(true);
                    setPathState(43);
                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();
        arm.update();
        viper.run();

        telemetry.addData("path state", pathState);
        telemetry.addData("state time", pathTimer.getElapsedTime());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("follower busy", follower.isBusy());
        telemetry.addData("arm busy", follower.isBusy());
        telemetry.addData("elapsed time < 1000 ", pathTimer.getElapsedTime() < 1000);
        telemetry.addData("viper current", ((DcMotorEx) viper.viper).getCurrent(CurrentUnit.AMPS));
        telemetry.addData("viper pos",  viper.viper.getCurrentPosition());
        telemetry.addData("arm degrees", arm.getCurrentPositionDegrees());
        telemetry.addData("intake pos", servos.intake.getPosition());
        telemetry.addData("wrist pos", servos.wrist.getPosition());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        viper  = new Slider("viper_motor", hardwareMap, DcMotor.Direction.REVERSE, true);
        arm    = new Arm    ("dc_arm", hardwareMap);
        servos = new Servos ("intake_servo", "wrist_servo", hardwareMap);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        servos.intakeCollect();
        servos.wristFolded();
        viper.calibrate();
        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
        telemetry.addData("Status", "Waiting for start");
        telemetry.update();
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}

