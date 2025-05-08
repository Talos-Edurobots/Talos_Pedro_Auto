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
    Pose startPose = new Pose(0, 0, Point.CARTESIAN);
    Pose endPose = new Pose(0, 0, Point.CARTESIAN);
    Path targetPose;

    @Override
    public void init() {
        setPathState(0);
        Constants.setConstants(FConstants.class, LConstants.class);
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void loop() {
        autonomousPathUpdate();
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
