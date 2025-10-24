import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import util.robot;

public class decodeAuto {
    public Follower follower;
    public Telemetry telemetry;

    public enum Side {
        RED,
        BLUE,
    }

    private robot R;
    public boolean actionBusy;


    private Side side;
    public Timer transferTimer = new Timer();
    public Timer specScoreTimer = new Timer();
    public Timer depositTimer = new Timer();
    public Timer postSpecScoreTimer = new Timer();
    public int transferState = -1, specimenNum = -1;
    public int depositState = -1;
    public int scoreSpecState = -1;
    public int postSpecScoreState = -1;
    public int postSpecScoreState2 = -1;
    public int fakeTransferState = -1;
    public int parkState = -1;
    public int extendSweepState = -1;
    public int extendRetractState = -1;

    public Pose startPose,
            shoot1Pose, center1Pose, release1Pose, releaseControl1Pose,
            pickup1Pose, pickup1Control1Pose, pickup2Pose, pickup2Control1Pose,
            pickup3Pose, pickup3Control1Pose, end1Pose;

    public Path scorePreload, end;

    public PathChain release1, shoot1, shoot2, shoot3;

    public Path[][] score = new Path[5][2];
    public int DISTANCE = 1;
    public decodeAuto(robot Robot, Telemetry telemetry, Follower follower, Side side) {


        this.follower = follower;
        this.telemetry = telemetry;
        this.side = side;

        createPose();
        buildPaths();

        init();
    }

    public void createPose() {
        switch (side) {
            case RED:
                startPose = new Pose(132, 61, Math.toRadians(0));
                shoot1Pose = new Pose(107, 108, Math.toRadians(-135));
                center1Pose = new Pose(79, 96, Math.toRadians(-270));
                release1Pose = new Pose(127, 72);
                releaseControl1Pose = new Pose(78, 70);
                pickup1Pose = new Pose(123, 84);
                pickup1Control1Pose = new Pose(47, 76);
                pickup2Pose = new Pose(123, 60);
                pickup2Control1Pose = new Pose(82, 56);
                pickup3Pose = new Pose(123, 36);
                pickup3Control1Pose = new Pose(79, 31);
                end1Pose = new Pose(107, 72);
                break;
            case BLUE:
                startPose = new Pose(11, 61, Math.toRadians(0));
                shoot1Pose = new Pose(36, 108, Math.toRadians(135));
                center1Pose = new Pose(64, 96, Math.toRadians(270));
                release1Pose = new Pose(16, 72);
                releaseControl1Pose = new Pose(65, 70);
                pickup1Pose = new Pose(20, 84);
                pickup1Control1Pose = new Pose(96, 76);
                pickup2Pose = new Pose(20, 60);
                pickup2Control1Pose = new Pose(61, 56);
                pickup3Pose = new Pose(20, 36);
                pickup3Control1Pose = new Pose(64, 31);
                end1Pose = new Pose(36, 72);
                break;
        }
    }

    public void buildPaths() {
        switch (side) {
            case RED:
            case BLUE:
        }
        release1 = follower.pathBuilder()
                .addPath(new BezierLine(shoot1Pose, center1Pose))
                .setLinearHeadingInterpolation(shoot1Pose.getHeading(), center1Pose.getHeading())
                .addPath(new BezierCurve(center1Pose, releaseControl1Pose, release1Pose))
                .setLinearHeadingInterpolation(center1Pose.getHeading(), release1Pose.getHeading())
                .build();

        shoot1 = follower.pathBuilder()
                .addPath(new BezierCurve(release1Pose, pickup1Control1Pose, pickup1Pose))
                .setLinearHeadingInterpolation(release1Pose.getHeading(), pickup1Pose.getHeading())
                .addPath(new BezierLine(pickup1Pose, shoot1Pose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), shoot1Pose.getHeading())
                .build();

        shoot2 = follower.pathBuilder()
                .addPath(new BezierLine(shoot1Pose, center1Pose))
                .setLinearHeadingInterpolation(shoot1Pose.getHeading(), center1Pose.getHeading())
                .addPath(new BezierCurve(center1Pose, pickup2Control1Pose, pickup2Pose))
                .setLinearHeadingInterpolation(center1Pose.getHeading(), pickup2Pose.getHeading())
                .addPath(new BezierLine(pickup2Pose, shoot1Pose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), shoot1Pose.getHeading())
                .build();

        shoot3 = follower.pathBuilder()
                .addPath(new BezierLine(shoot1Pose, center1Pose))
                .setLinearHeadingInterpolation(shoot1Pose.getHeading(), center1Pose.getHeading())
                .addPath(new BezierCurve(center1Pose, pickup3Control1Pose, pickup3Pose))
                .setLinearHeadingInterpolation(center1Pose.getHeading(), pickup3Pose.getHeading())
                .addPath(new BezierLine(pickup3Pose, shoot1Pose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), shoot1Pose.getHeading())
                .build();

        scorePreload = new Path(new BezierLine(startPose, shoot1Pose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), shoot1Pose.getHeading());
        end = new Path(new BezierLine(shoot1Pose, end1Pose));
        end.setLinearHeadingInterpolation(shoot1Pose.getHeading(), end1Pose.getHeading());
    }

    public void intakeBalls(int numBalls) {
        R.intakeLow.setPower(1);
        Thread.sleep(numBalls * 500);
        R.intakeLow.setPower(0);
    }

    public void sendBallsToShooter(int numBalls) {
        R.intakeHigh.setPower(1);
        Thread.sleep(numBalls * 500);
        R.intakeHigh.setPower(0);
    }

    public void shoot(int numBalls) {
        R.shooter.setPower(1);
        Thread.sleep(numBalls * 500);
        R.shooter.setPower(0);
    }

    public void init() {

    }

    public void start() {

    }

    public void update() {
        follower.update();

        park();
    }
}