package pedroPathing.examples;

import com.pedropathing.pathgen.BezierCurve;
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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/** This is the program for the autonomous period on the Observation Zone for this year.
 * */
@Autonomous(name="Red Observation Autonomous", group="Examples")
public class TalosObservationAutonomous extends OpMode {
    /** We create a Follower object from PedroPathing*/
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private ElapsedTime timer;
    private int pathState;
    Arm    arm;
    Servos servos;
    Viper  viper;
    private final Pose startPose    = new Pose(0,  70, Math.toRadians(0));
    private final Pose scorePreloadPose = new Pose(28.75, 70, Math.toRadians(0));
    private final Pose controlPoint1 = new Pose(4.5, 44, Math.toRadians(0));
    private final Pose samplesPose = new Pose(50.3, 37, Math.toRadians(0));
    private final Pose sample1Pose = new Pose(50.3, 33, Math.toRadians(0));
    private final Pose attach1Pose = new Pose(14, 33, Math.toRadians(0));
    private final Pose sample2Pose = new Pose(50.3, 24, Math.toRadians(0));
    private final Pose attach2Pose = new Pose(14, 27, Math.toRadians(0));
    private final Pose sample3Pose = new Pose(50.3, 18, Math.toRadians(0));
    private final Pose attach3Pose = new Pose(10, 18, Math.toRadians(0));
    private final Pose controlPoint2 = new Pose(40, 20, Math.toRadians(0));
    private final Pose preGrabPose = new Pose(12, 30, Math.toRadians(180));
    private final Pose grabSpecimenPose = new Pose(2, 30, Math.toRadians(180));
    private final Pose scoreFirstPose = new Pose(28.75, 72, Math.toRadians(0));


    private PathChain attachSamples;
    private Path scorePreload;
    private PathChain grabFirst;
    private Path scoreFirst;
    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePreloadPose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePreloadPose.getHeading());
        attachSamples = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePreloadPose), new Point(controlPoint1), new Point(samplesPose)))
                .setLinearHeadingInterpolation(scorePreloadPose.getHeading(), samplesPose.getHeading())

                .addPath(new BezierLine(new Point(samplesPose), new Point(sample1Pose)))
                .setLinearHeadingInterpolation(samplesPose.getHeading(), sample1Pose.getHeading())

                .addPath(new BezierLine(new Point(sample1Pose), new Point(attach1Pose)))
                .setLinearHeadingInterpolation(sample1Pose.getHeading(), attach1Pose.getHeading())
                .setPathEndVelocityConstraint(.5)

                .addPath(new BezierLine(new Point(attach1Pose), new Point(sample1Pose)))
                .setLinearHeadingInterpolation(attach1Pose.getHeading(), sample1Pose.getHeading())

                .addPath(new BezierLine(new Point(sample1Pose), new Point(sample2Pose)))
                .setLinearHeadingInterpolation(sample1Pose.getHeading(), sample2Pose.getHeading())

                .addPath(new BezierLine(new Point(sample2Pose), new Point(attach2Pose)))
                .setLinearHeadingInterpolation(sample2Pose.getHeading(), attach2Pose.getHeading())
                .setPathEndVelocityConstraint(.5)

                .addPath(new BezierLine(new Point(attach2Pose), new Point(sample2Pose)))
                .setLinearHeadingInterpolation(attach2Pose.getHeading(), sample2Pose.getHeading())

                .addPath(new BezierLine(new Point(sample2Pose), new Point(sample3Pose)))
                .setLinearHeadingInterpolation(sample2Pose.getHeading(), sample3Pose.getHeading())

                .addPath(new BezierLine(new Point(sample3Pose), new Point(attach3Pose)))
                .setLinearHeadingInterpolation(sample3Pose.getHeading(), attach3Pose.getHeading())
                .setPathEndVelocityConstraint(.5)
                
                .build();


        grabFirst = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(attach3Pose), new Point(controlPoint2), new Point(preGrabPose)))
                .setLinearHeadingInterpolation(attach3Pose.getHeading(), preGrabPose.getHeading())

                .addPath(new BezierLine(new Point(preGrabPose), new Point(grabSpecimenPose)))
                .setLinearHeadingInterpolation(preGrabPose.getHeading(), grabSpecimenPose.getHeading())

                .build();

        scoreFirst = new Path(new BezierLine(new Point(grabSpecimenPose), new Point(scoreFirstPose)));
        scoreFirst.setLinearHeadingInterpolation(grabSpecimenPose.getHeading(), scoreFirstPose.getHeading());
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                arm.setPositionDegrees(90);
                setPathState(1);
                break;
            case 1:
                if (!(follower.isBusy() || arm.arm.isBusy())) {
                    arm.setPositionDegrees(50);
                    setPathState(2);
                }
                break;
            case 2:
                if (!arm.arm.isBusy()) {
                    servos.intakeOpen();
                    follower.followPath(attachSamples, true);
                    arm.setPositionDegrees(15);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    arm.setPositionDegrees(30);
                    follower.followPath(grabFirst, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!(follower.isBusy()  || arm.arm.isBusy())) {
                    servos.intakeCollect();
                    setPathState(5);
                }
                break;
            case 5:
                if (pathTimer.getElapsedTime() > 1000) {
                    arm.setPositionDegrees(50);
                    follower.followPath(scoreFirst);
                }
                break;
            case 6:
                if (!(follower.isBusy() || arm.arm.isBusy())) {
                    arm.setPositionDegrees(90);
                    setPathState(6);
                }
                break;
            case 7:
                if (!arm.arm.isBusy()) {
                    servos.intakeOpen();
                    arm.setPositionDegrees(30);
                    setPathState(7);
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
        viper  = new Viper  (this, "viper_motor", hardwareMap, telemetry);
        servos = new Servos ("intake_servo", "wrist_servo", hardwareMap);
        telemetry.addData("Status", "init");
        telemetry.update();
        telemetry.addData("Status", "after_init");
        telemetry.update();
        servos.intakeCollect();
        servos.wristHorizontal();
        arm.setPositionDegrees(10);
        arm.runArm(false);
        viper.calibrateViper();

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
        arm.runArm(false);
        viper.runViper(false);


        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("follower busy", follower.isBusy());
        telemetry.addData("viper current", ((DcMotorEx) viper.viper).getCurrent(CurrentUnit.AMPS));
        telemetry.addData("viper pos",  viper.viper.getCurrentPosition());
        telemetry.update();
    }
    @Override
    public void stop() {
        viper.setPositionTicks(0);
        arm.setPositionDegrees(0);
        viper.runViper(false);
        arm.runArm(false);
    }
}
