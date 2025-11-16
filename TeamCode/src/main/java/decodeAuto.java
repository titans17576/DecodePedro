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
        REDCLOSE,
        BLUECLOSE,
        REDFAR,
        BLUEFAR
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
            shoot1Pose, center1Pose, center2Pose, center3Pose, release1Pose, releaseControl1Pose,
            pickup1Pose, pickup1Control1Pose, pickup2Pose, pickup2Control1Pose,
            pickup3Pose, pickup3Control1Pose, end1Pose,
            shootFar1Pose, turnHPZone1Pose, pickupHPZone1Pose, turnPickupFar1Pose, turnPickupFar1ControlPose, pickupFar1Pose, farParkPose;
    ;

    //public Path scorePreload, end;

    public PathChain release1, shootPreload, shoot1, shoot2, shoot3, scorePreload, end, intake1, intake2;
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
            case REDCLOSE:
                startPose = new Pose(124, 120, Math.toRadians(36));
                shoot1Pose = new Pose(96, 96, Math.toRadians(45));
                center1Pose = new Pose(84, 96, Math.toRadians(90));
                center2Pose = new Pose(84, 96, Math.toRadians(90));
                center3Pose = new Pose(84, 96, Math.toRadians(90));
                release1Pose = new Pose(128, 72, Math.toRadians(0));
                releaseControl1Pose = new Pose(79, 70);
                pickup1Pose = new Pose(124, 84, Math.toRadians(0));
                pickup2Pose = new Pose(124, 60, Math.toRadians(0));
                pickup3Pose = new Pose(124, 36, Math.toRadians(0));
                end1Pose = new Pose(108, 72, Math.toRadians(0));
                break;
            case BLUECLOSE:
                startPose = new Pose(21, 123, Math.toRadians(138));
                shoot1Pose = new Pose(48, 96, Math.toRadians(133));
                center1Pose = new Pose(45, 84, Math.toRadians(180));
                center2Pose = new Pose(45, 59, Math.toRadians(180));
                center3Pose = new Pose(45, 36, Math.toRadians(180));
                release1Pose = new Pose(16, 72, Math.toRadians(180));
                releaseControl1Pose = new Pose(65, 70);
                pickup1Pose = new Pose(24, 84, Math.toRadians(180));
                pickup2Pose = new Pose(11, 59, Math.toRadians(180));
                pickup3Pose = new Pose(11, 36, Math.toRadians(180));
                end1Pose = new Pose(36, 72, Math.toRadians(180));
                break;
            case BLUEFAR:
                startPose = new Pose(60, 8, Math.toRadians(90));
                shootFar1Pose = new Pose(60, 12,Math.toRadians(116));
                turnHPZone1Pose = new Pose(28, 12, Math.toRadians(180));
                pickupHPZone1Pose = new Pose(10, 12, Math.toRadians(180));
                turnPickupFar1Pose = new Pose(45, 33, Math.toRadians(180));
                turnPickupFar1ControlPose = new Pose(54, 32);
                pickupFar1Pose = new Pose(14, 33, Math.toRadians(180));
                farParkPose = new Pose(60, 36, Math.toRadians(90));
                break;
        }
    }

    public void buildPaths() {
        switch (side) {
            case BLUECLOSE:
                release1 = follower.pathBuilder()
                    .addPath(new BezierLine(shoot1Pose, center1Pose))
                    .setLinearHeadingInterpolation(shoot1Pose.getHeading(), center1Pose.getHeading())
                    .addPath(new BezierCurve(center1Pose, releaseControl1Pose, release1Pose))
                    .setLinearHeadingInterpolation(center1Pose.getHeading(), release1Pose.getHeading())
                    .build();

                shoot1 = follower.pathBuilder()
                    .addPath(new BezierLine(shoot1Pose, center1Pose))
                    .setLinearHeadingInterpolation(shoot1Pose.getHeading(), center1Pose.getHeading())
                    .addPath(new BezierLine(center1Pose, pickup1Pose))
                    .setConstantHeadingInterpolation(pickup1Pose.getHeading())
                    .addPath(new BezierLine(pickup1Pose, shoot1Pose))
                    .setLinearHeadingInterpolation(pickup1Pose.getHeading(), shoot1Pose.getHeading())
                    .build();

                shoot2 = follower.pathBuilder()
                    .addPath(new BezierLine(shoot1Pose, center2Pose))
                    .setLinearHeadingInterpolation(shoot1Pose.getHeading(), center2Pose.getHeading())
                    .addPath(new BezierLine(center2Pose, pickup2Pose))
                    .setConstantHeadingInterpolation(pickup2Pose.getHeading())
                    .addPath(new BezierLine(pickup2Pose, center2Pose))
                    .setConstantHeadingInterpolation(pickup2Pose.getHeading())
                    .addPath(new BezierLine(center2Pose, shoot1Pose))
                    .setLinearHeadingInterpolation(center2Pose.getHeading(), shoot1Pose.getHeading())
                    .build();

                shoot3 = follower.pathBuilder()
                    .addPath(new BezierLine(shoot1Pose, center3Pose))
                    .setLinearHeadingInterpolation(shoot1Pose.getHeading(), center3Pose.getHeading())
                    .addPath(new BezierLine(center3Pose, pickup3Pose))
                    .setConstantHeadingInterpolation(pickup1Pose.getHeading())
                    .addPath(new BezierLine(pickup3Pose, shoot1Pose))
                    .setLinearHeadingInterpolation(pickup3Pose.getHeading(), shoot1Pose.getHeading())
                    .build();

                shootPreload = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, shoot1Pose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), shoot1Pose.getHeading())
                    .build();

                end = follower.pathBuilder()
                    .addPath(new BezierLine(shoot1Pose, end1Pose))
                    .setLinearHeadingInterpolation(shoot1Pose.getHeading(), end1Pose.getHeading())
                    .build();
                break;
            case BLUEFAR:
                shoot1 = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, shootFar1Pose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), shootFar1Pose.getHeading())
                    .build();

                intake1 = follower.pathBuilder()
                    .addPath(new BezierLine(shootFar1Pose, turnHPZone1Pose))
                    .setLinearHeadingInterpolation(shootFar1Pose.getHeading(), turnHPZone1Pose.getHeading())
                    .addPath(new BezierLine(turnHPZone1Pose, pickupHPZone1Pose))
                    .setConstantHeadingInterpolation(pickupHPZone1Pose.getHeading())
                    .build();

                shoot2 = follower.pathBuilder()
                    .addPath(new BezierLine(pickupHPZone1Pose, shootFar1Pose))
                    .setLinearHeadingInterpolation(pickupHPZone1Pose.getHeading(), shootFar1Pose.getHeading())
                    .build();

                intake2 = follower.pathBuilder()
                    .addPath(new BezierCurve(shootFar1Pose, turnPickupFar1ControlPose, turnPickupFar1Pose))
                    .setLinearHeadingInterpolation(shootFar1Pose.getHeading(), turnPickupFar1Pose.getHeading())
                    .addPath(new BezierLine(turnPickupFar1Pose, pickupFar1Pose))
                    .setConstantHeadingInterpolation(pickupFar1Pose.getHeading())
                    .build();
                shoot3 = follower.pathBuilder()
                    .addPath(new BezierLine(pickupFar1Pose, shootFar1Pose))
                    .setLinearHeadingInterpolation(pickupFar1Pose.getHeading(), shootFar1Pose.getHeading())
                    .build();
                end = follower.pathBuilder()
                    .addPath(new BezierLine(shootFar1Pose, farParkPose))
                    .setLinearHeadingInterpolation(shootFar1Pose.getHeading(), farParkPose.getHeading())
                    .build();
                break;
        }
    }

    public void intakeBalls() {
        switch(intakeState){
            case 1:
                IntakeFSM.setLowIntakeState(intakeFSM.LowIntakeState.ON);
                //IntakeFSM.setGatekeepState(intakeFSM.GatekeepState.ON);
                intakeTimer.resetTimer();
                setIntakeState(2);
                break;
            case 2:
                if (intakeTimer.getElapsedTimeSeconds() > 2.5) {
                    IntakeFSM.setLowIntakeState(intakeFSM.LowIntakeState.OFF);
                    //IntakeFSM.setGatekeepState(intakeFSM.GatekeepState.OFF);
                    setIntakeState(-1);
                }
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
                    IntakeFSM.setLowIntakeState(intakeFSM.LowIntakeState.ON);
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
                    IntakeFSM.setLowIntakeState(intakeFSM.LowIntakeState.OFF);
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