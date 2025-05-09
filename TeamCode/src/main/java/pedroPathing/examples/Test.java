package pedroPathing.examples;

import com.pedropathing.pathgen.BezierCurve;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.actuators.Arm;
import pedroPathing.actuators.Servos;
import pedroPathing.actuators.GobildaViper;

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

@Autonomous(name="my test")
public class Test extends OpMode {
    Follower follower;
    int pathState;
    private Timer pathTimer, actionTimer, opmodeTimer;
    Pose startPose = new Pose(0, 0, 0);
    Pose endPose = new Pose(0, 0, Math.toRadians(180));
    Path targetPose;

    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        setPathState(0);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void loop() {
        autonomousPathUpdate();
        follower.update();

        telemetry.addData("path state", pathState);
        telemetry.addData("state time", pathTimer.getElapsedTime());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("follower busy", follower.isBusy());
        telemetry.addData("arm busy", follower.isBusy());
        telemetry.addData("elapsed time < 1000 ", pathTimer.getElapsedTime() < 1000);
        telemetry.addData("follower error", follower.driveError);
        telemetry.update();
    }
    public void buildPaths() {
        targetPose = new Path(new BezierLine(new Point(startPose), new Point(endPose)));
        targetPose.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180));
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(targetPose);
                setPathState(1);
                break;
        }
    }
}
