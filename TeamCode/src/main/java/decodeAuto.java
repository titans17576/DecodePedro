import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import pedroPathing.constants.Constants;
import util.robot;

public class decodeAuto {
    public Follower follower;
    public Telemetry telemetry;

    public enum Side {
        RED,
        BLUE,
    }

    public robot R;
    public boolean actionBusy;
    public intakeFSM IntakeFSM;


    private Side side;
    public Timer shootTimer = new Timer();
    public Timer intakeTimer = new Timer();
    public int shootState = -1;
    public int intakeState = -1;

    public Pose startPose,
            shoot1Pose, center1Pose, release1Pose, releaseControl1Pose,
            pickup1Pose, pickup1Control1Pose, pickup2Pose, pickup2Control1Pose,
            pickup3Pose, pickup3Control1Pose, end1Pose;

    //public Path scorePreload, end;

    public PathChain release1, shoot1, shoot2, shoot3, scorePreload, end;
    public decodeAuto(robot Robot, Telemetry telemetry, Follower follower, Side side) {
        IntakeFSM = new intakeFSM(Robot, telemetry);

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
                startPose = new Pose(108, 136, Math.toRadians(270));
                shoot1Pose = new Pose(96, 96, Math.toRadians(225));
                center1Pose = new Pose(79, 96, Math.toRadians(90));
                release1Pose = new Pose(127, 72, Math.toRadians(0));
                releaseControl1Pose = new Pose(78, 70);
                pickup1Pose = new Pose(123, 84, Math.toRadians(0));
                pickup1Control1Pose = new Pose(47, 76);
                pickup2Pose = new Pose(123, 60, Math.toRadians(0));
                pickup2Control1Pose = new Pose(82, 56);
                pickup3Pose = new Pose(123, 36, Math.toRadians(0));
                pickup3Control1Pose = new Pose(79, 31);
                end1Pose = new Pose(107, 72, Math.toRadians(0));
                break;
            case BLUE:
                startPose = new Pose(36, 136, Math.toRadians(270));
                shoot1Pose = new Pose(48, 96, Math.toRadians(135));
                center1Pose = new Pose(64, 96, Math.toRadians(270));
                release1Pose = new Pose(16, 72, Math.toRadians(180));
                releaseControl1Pose = new Pose(65, 70);
                pickup1Pose = new Pose(20, 84, Math.toRadians(180));
                pickup1Control1Pose = new Pose(50, 82);
                pickup2Pose = new Pose(20, 60, Math.toRadians(180));
                pickup2Control1Pose = new Pose(61, 56);
                pickup3Pose = new Pose(20, 36, Math.toRadians(180));
                pickup3Control1Pose = new Pose(64, 31);
                end1Pose = new Pose(36, 72, Math.toRadians(180));
                break;
        }
    }

    public void buildPaths() {
        release1 = follower.pathBuilder()
                .addPath(new BezierLine(shoot1Pose, center1Pose))
                .setLinearHeadingInterpolation(shoot1Pose.getHeading(), center1Pose.getHeading())
                .addPath(new BezierCurve(center1Pose, releaseControl1Pose, release1Pose))
                .setLinearHeadingInterpolation(center1Pose.getHeading(), release1Pose.getHeading())
                .build();

        shoot1 = follower.pathBuilder()
                .addPath(new BezierLine(shoot1Pose, center1Pose))
                .setLinearHeadingInterpolation(shoot1Pose.getHeading(), center1Pose.getHeading())
                .addPath(new BezierCurve(center1Pose, pickup1Control1Pose, pickup1Pose))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(pickup1Pose, shoot1Pose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), shoot1Pose.getHeading())
                .build();

        shoot2 = follower.pathBuilder()
                .addPath(new BezierLine(shoot1Pose, center1Pose))
                .setLinearHeadingInterpolation(shoot1Pose.getHeading(), center1Pose.getHeading())
                .addPath(new BezierCurve(center1Pose, pickup2Control1Pose, pickup2Pose))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(pickup2Pose, shoot1Pose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), shoot1Pose.getHeading())
                .build();

        shoot3 = follower.pathBuilder()
                .addPath(new BezierLine(shoot1Pose, center1Pose))
                .setLinearHeadingInterpolation(shoot1Pose.getHeading(), center1Pose.getHeading())
                .addPath(new BezierCurve(center1Pose, pickup3Control1Pose, pickup3Pose))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(pickup3Pose, shoot1Pose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), shoot1Pose.getHeading())
                .build();

        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shoot1Pose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shoot1Pose.getHeading())
                .build();

        end = follower.pathBuilder()
                .addPath(new BezierLine(shoot1Pose, end1Pose))
                .setLinearHeadingInterpolation(shoot1Pose.getHeading(), end1Pose.getHeading())
                .build();
    }

    public void intakeBalls() {
        switch(intakeState){
            case 1:
                IntakeFSM.setHighIntakeState(intakeFSM.HighIntakeState.ON);
                IntakeFSM.setLowIntakeState(intakeFSM.LowIntakeState.ON);
                intakeTimer.resetTimer();
                setIntakeState(2);
                break;
            case 2:
                if (intakeTimer.getElapsedTimeSeconds() > 2) {
                    IntakeFSM.setLowIntakeState(intakeFSM.LowIntakeState.OFF);
                    IntakeFSM.setHighIntakeState(intakeFSM.HighIntakeState.OFF);
                }
                setIntakeState(-1);
                break;
        }
    }

    public void shoot() {
        switch(shootState){
            case 1:
                actionBusy = true;
                IntakeFSM.setHighIntakeState(intakeFSM.HighIntakeState.ON);
                shootTimer.resetTimer();
                setShootState(2);
                break;
            case 2:
                if (shootTimer.getElapsedTimeSeconds() > 0.5) {
                    IntakeFSM.setHighIntakeState(intakeFSM.HighIntakeState.OFF);
                    shootTimer.resetTimer();
                    setShootState(3);
                }
                break;
            case 3:
                if (shootTimer.getElapsedTimeSeconds() > 0.5) {
                    IntakeFSM.setHighIntakeState(intakeFSM.HighIntakeState.ON);
                    shootTimer.resetTimer();
                    setShootState(4);
                }
                break;
            case 4:
                if (shootTimer.getElapsedTimeSeconds() > 0.5) {
                    IntakeFSM.setHighIntakeState(intakeFSM.HighIntakeState.OFF);
                    shootTimer.resetTimer();
                    setShootState(5);
                }
                break;
            case 5:
                if (shootTimer.getElapsedTimeSeconds() > 0.5) {
                    IntakeFSM.setHighIntakeState(intakeFSM.HighIntakeState.ON);
                    shootTimer.resetTimer();
                    setShootState(6);
                }
                break;
            case 6:
                if (shootTimer.getElapsedTimeSeconds() > 0.5) {
                    IntakeFSM.setHighIntakeState(intakeFSM.HighIntakeState.OFF);
                    shootTimer.resetTimer();
                    setShootState(7);
                }
                break;
            case 7:
                actionBusy = false;
                setShootState(-1);
                break;
        }
    }

    public void init() {

    }

    public void start() {

    }

    public void update() {
        follower.update();
        IntakeFSM.update();
        shoot();
        intakeBalls();
    }
    public void setShootState(int x){
        shootState = x;
        telemetry.addData("Shoot", x);
    }
    public void setIntakeState(int x){
        intakeState = x;
        telemetry.addData("Intake", x);
    }
    public void startShoot(){
        if (!actionBusy) {
            setShootState(1);
        }
    }
    public void startIntake(){
        if (!actionBusy) {
            setIntakeState(1);
        }
    }
    public boolean actionNotBusy() {
        return !actionBusy;
    }

    public boolean notBusy() {
        return (!follower.isBusy() && actionNotBusy());
    }
}