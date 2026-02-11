import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import util.robot;

public class autoConfig {
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
            shoot1Pose, center1Pose, center2Pose, center3Pose, release1Pose, release2Pose, releaseControl1Pose, releaseControl2Pose,
            pickup1Pose, pickup1Control1Pose, pickup2Pose, pickup2Control1Pose, pickup2Control2Pose,
            pickup3Pose, pickup3Control1Pose, pickup3Control2Pose, end1Pose, endShootPose,
            return2Pose,
            shootFar1Pose, turnHPZone1Pose, pickupHPZone1Pose, turnPickupFar1Pose, turnPickupFar1ControlPose, pickupFar1Pose, farParkPose;
    ;

    //public Path scorePreload, end;

    public PathChain release1, shootPreload, shoot1, shoot2, shoot3, scorePreload, end, intake1, intake2, intake3, shootGate;
    public autoConfig(robot Robot, Telemetry telemetry, Follower follower, Side side) {
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
                startPose = new Pose(123, 123, Math.toRadians(42)); //mirrored field is 141.5 inches - blue, not 144 inches
                shoot1Pose = new Pose(96, 96, Math.toRadians(47));
                center1Pose = new Pose(99, 84, Math.toRadians(0));
                center2Pose = new Pose(99, 59, Math.toRadians(0));
                center3Pose = new Pose(99, 37, Math.toRadians(0));
                release1Pose = new Pose(129, 62, Math.toRadians(0));
                release2Pose = new Pose(134, 62, Math.toRadians(33));
                releaseControl1Pose = new Pose(94, 78);
                pickup1Pose = new Pose(123, 84, Math.toRadians(0));
                pickup2Pose = new Pose(130, 59, Math.toRadians(0));
                return2Pose = new Pose(106, 59, Math.toRadians(0));
                pickup3Pose = new Pose(132, 37, Math.toRadians(0));
                end1Pose = new Pose(108, 72, Math.toRadians(0));
                endShootPose = new Pose(108, 108, Math.toRadians(37));
                break;
            case BLUECLOSE:
                startPose = new Pose(21, 123, Math.toRadians(138));
                shoot1Pose = new Pose(48, 96, Math.toRadians(133));
                release2Pose = new Pose(12,61, Math.toRadians(160)); //was 6, 61
                releaseControl1Pose = new Pose(45, 56);
                pickup1Pose = new Pose(20, 84, Math.toRadians(180));
                pickup1Control1Pose = new Pose(45, 82);
                pickup2Pose = new Pose(18, 60, Math.toRadians(190)); //was 16, 180 degrees
                pickup2Control1Pose = new Pose(52, 55); //was 58, 55
                pickup2Control2Pose = new Pose(43, 60);
                pickup3Pose = new Pose(16, 36, Math.toRadians(180)); //was 12
                pickup3Control1Pose = new Pose(55, 26);
                pickup3Control2Pose = new Pose(43, 35);
                end1Pose = new Pose(52, 104, Math.toRadians(125));
                endShootPose = new Pose(40, 108, Math.toRadians(145));
                break;
            case BLUEFAR:
                startPose = new Pose(60, 8, Math.toRadians(90));
                shootFar1Pose = new Pose(58, 16,Math.toRadians(116));
                turnHPZone1Pose = new Pose(28, 12, Math.toRadians(180));
                pickupHPZone1Pose = new Pose(10, 12, Math.toRadians(180));
                turnPickupFar1Pose = new Pose(45, 33, Math.toRadians(180));
                turnPickupFar1ControlPose = new Pose(54, 32);
                pickupFar1Pose = new Pose(14, 33, Math.toRadians(180));
                farParkPose = new Pose(60, 36, Math.toRadians(90));
                break;
            case REDFAR:
                startPose = new Pose(84, 8, Math.toRadians(90));
                shootFar1Pose = new Pose(86, 16,Math.toRadians(64));
                turnHPZone1Pose = new Pose(28, 12, Math.toRadians(0));
                pickupHPZone1Pose = new Pose(10, 12, Math.toRadians(0));
                turnPickupFar1Pose = new Pose(45, 33, Math.toRadians(0));
                turnPickupFar1ControlPose = new Pose(54, 32);
                pickupFar1Pose = new Pose(14, 33, Math.toRadians(0));
                farParkPose = new Pose(60, 36, Math.toRadians(90));
                break;
        }
    }

    public void buildPaths() {
        switch (side) {
            case BLUECLOSE:
                shootPreload = follower.pathBuilder()
                        .addPath(new BezierLine(startPose, shoot1Pose))
                        .setLinearHeadingInterpolation(startPose.getHeading(), shoot1Pose.getHeading())
                        .build();

                release1 = follower.pathBuilder()
                        .addPath(new BezierCurve(shoot1Pose, releaseControl1Pose, release2Pose))
                        .setConstantHeadingInterpolation(shoot1Pose.getHeading())
                        .build();

                shootGate = follower.pathBuilder()
                        .addPath(new BezierCurve(release2Pose, releaseControl1Pose, shoot1Pose))
                        .setConstantHeadingInterpolation(shoot1Pose.getHeading())
                        .build();

                intake1 = follower.pathBuilder()
                        .addPath(new BezierCurve(shoot1Pose, pickup1Control1Pose, pickup1Pose))
                        .setLinearHeadingInterpolation(shoot1Pose.getHeading(), pickup1Pose.getHeading())
                        .build();

                shoot1 = follower.pathBuilder()
                        .addPath(new BezierLine(pickup1Pose, shoot1Pose))
                        .setLinearHeadingInterpolation(pickup1Pose.getHeading(), shoot1Pose.getHeading())
                        .build();

                intake2 = follower.pathBuilder()
                        .addPath(new BezierCurve(shoot1Pose, pickup2Control1Pose, pickup2Control2Pose, pickup2Pose))
                        .setLinearHeadingInterpolation(shoot1Pose.getHeading(), pickup2Pose.getHeading())
                        .build();

                shoot2 = follower.pathBuilder()
                        .addPath(new BezierCurve(pickup2Pose, pickup2Control2Pose, pickup2Control1Pose, shoot1Pose))
                        .setConstantHeadingInterpolation(shoot1Pose.getHeading())
                        .build();

                intake3 = follower.pathBuilder()
                        .addPath(new BezierCurve(shoot1Pose, pickup3Control1Pose, pickup3Control2Pose, pickup3Pose))
                        .setLinearHeadingInterpolation(shoot1Pose.getHeading(), pickup3Pose.getHeading())
                        .build();

                shoot3 = follower.pathBuilder()
                        .addPath(new BezierLine(pickup3Pose, shoot1Pose))
                        .setConstantHeadingInterpolation(shoot1Pose.getHeading())
                        .build();

                end = follower.pathBuilder()
                        .addPath(new BezierLine(shoot1Pose, end1Pose))
                        .setConstantHeadingInterpolation(end1Pose.getHeading())
                        .build();
                break;
            case REDCLOSE:
                release1 = follower.pathBuilder()
                        .addPath(new BezierLine(shoot1Pose, return2Pose))
                        .setLinearHeadingInterpolation(shoot1Pose.getHeading(), return2Pose.getHeading())
                        .addPath(new BezierLine(return2Pose, release1Pose))
                        .setLinearHeadingInterpolation(return2Pose.getHeading(), release1Pose.getHeading())
                        .addPath(new BezierLine(release1Pose, release2Pose))
                        .setLinearHeadingInterpolation(release1Pose.getHeading(), release2Pose.getHeading())
                        .build();

                shootGate = follower.pathBuilder()
                        .addPath(new BezierLine(release2Pose, center2Pose))
                        .setLinearHeadingInterpolation(release2Pose.getHeading(), center2Pose.getHeading())
                        .addPath(new BezierLine(center2Pose, shoot1Pose))
                        .setLinearHeadingInterpolation(center2Pose.getHeading(), shoot1Pose.getHeading())
                        .build();

                intake1 = follower.pathBuilder()
                        .addPath(new BezierLine(shoot1Pose, center1Pose))
                        .setLinearHeadingInterpolation(shoot1Pose.getHeading(), center1Pose.getHeading())
                        .addPath(new BezierLine(center1Pose, pickup1Pose))
                        .setConstantHeadingInterpolation(pickup1Pose.getHeading())
                        .build();

                shoot1 = follower.pathBuilder()
                        .addPath(new BezierLine(pickup1Pose, shoot1Pose))
                        .setLinearHeadingInterpolation(pickup1Pose.getHeading(), shoot1Pose.getHeading())
                        .build();

                intake2 = follower.pathBuilder()
                        .addPath(new BezierLine(shoot1Pose, center2Pose))
                        .setLinearHeadingInterpolation(shoot1Pose.getHeading(), center2Pose.getHeading())
                        .addPath(new BezierLine(center2Pose, pickup2Pose))
                        .setConstantHeadingInterpolation(pickup2Pose.getHeading())
                        .build();

                shoot2 = follower.pathBuilder()
                        .addPath(new BezierLine(pickup2Pose, return2Pose))
                        .setConstantHeadingInterpolation(return2Pose.getHeading())
                        .addPath(new BezierLine(return2Pose, shoot1Pose))
                        .setConstantHeadingInterpolation(shoot1Pose.getHeading())
                        .build();

                intake3 = follower.pathBuilder()
                        .addPath(new BezierLine(shoot1Pose, center3Pose))
                        .setLinearHeadingInterpolation(shoot1Pose.getHeading(), center3Pose.getHeading())
                        .addPath(new BezierLine(center3Pose, pickup3Pose))
                        .setConstantHeadingInterpolation(pickup1Pose.getHeading())
                        .build();

                shoot3 = follower.pathBuilder()
                        .addPath(new BezierLine(pickup3Pose, shoot1Pose))
                        .setConstantHeadingInterpolation(shoot1Pose.getHeading())
                        .build();

                shootPreload = follower.pathBuilder()
                        .addPath(new BezierLine(startPose, shoot1Pose))
                        .setLinearHeadingInterpolation(startPose.getHeading(), shoot1Pose.getHeading())
                        .build();

                end = follower.pathBuilder()
                        .addPath(new BezierLine(shoot1Pose, end1Pose))
                        .setConstantHeadingInterpolation(end1Pose.getHeading())
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
            case REDFAR:
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

    public void intake() {
        switch(intakeState){
            case 1:
                IntakeFSM.setLowIntakeState(intakeFSM.LowIntakeState.ON);
                IntakeFSM.setHighIntakeState(intakeFSM.HighIntakeState.ON);
                intakeTimer.resetTimer();
                setIntakeState(2);
                break;
            case 2:
                if (intakeTimer.getElapsedTimeSeconds() > 1) {
                    IntakeFSM.setGatekeepState(intakeFSM.GatekeepState.ON);
                    setIntakeState(-1);
                }
                break;
            case 3:
                IntakeFSM.setLowIntakeState(intakeFSM.LowIntakeState.OFF);
                IntakeFSM.setHighIntakeState(intakeFSM.HighIntakeState.OFF);
                setIntakeState(-1);
                break;
        }
    }

    public void shoot() {
        switch(shootState){
            case 1:
                IntakeFSM.setGatekeepState(intakeFSM.GatekeepState.OFF);
                actionBusy = true;
                IntakeFSM.setHighIntakeState(intakeFSM.HighIntakeState.ON);
                IntakeFSM.setLowIntakeState(intakeFSM.LowIntakeState.ON);
                shootTimer.resetTimer();
                setShootState(2);
                break;
            case 2:
                if (shootTimer.getElapsedTimeSeconds() > 0.7) {
                    IntakeFSM.setHighIntakeState(intakeFSM.HighIntakeState.OFF);
                    IntakeFSM.setLowIntakeState(intakeFSM.LowIntakeState.OFF);
                    setShootState(3); //rapid fire?
                }
                break;
            case 3:
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
        intake();
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
    public void stopIntake(){
        if (!actionBusy) {
            setIntakeState(3);
        }
    }
    public boolean actionNotBusy() {
        return !actionBusy;
    }

    public boolean notBusy() {
        return (!follower.isBusy() && actionNotBusy());
    }
}