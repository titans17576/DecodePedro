import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import util.robot;

public class Auto {
    public Follower follower;
    public Telemetry telemetry;
    public enum Side{
        BUCKET,
        OBSERVATION,
    }

    private robot R;
    public boolean actionBusy;

    public liftFSM LiftFSM;
    public clawFSM ClawFSM;
    public scoopFSM ScoopFSM;

    private Side side;
    public Timer transferTimer = new Timer();
    public Timer depositTimer = new Timer();
    public int transferState = -1, specimenNum = -1;
    public int depositState = -1;
    public Path forwards, backwards;


    public Pose startPose,
            specimen1Pose,specimen2Pose, specimen3Pose,
            shortBack1Pose, longBack2Pose, longBack3Pose, longBack4Pose,
            shift2Pose, shift3Pose, shift4Pose,
            pickup2Pose, pickup3Pose, pickup4Pose,
            specimenControlPoint1Pose, specimenControlPoint2Pose,
            curveControlPoint1Pose, curveControlPoint2Pose;


    public PathChain moveCurve, push23, gather3, goal2, goal3;
    public Path goal1;

    public Path[][] score = new Path[5][2];
    public int DISTANCE = 1;

    public Auto(robot Robot, Telemetry telemetry, Follower follower, Side side) {
        ClawFSM = new clawFSM(Robot, telemetry);
        LiftFSM = new liftFSM(Robot, telemetry);
        ScoopFSM = new scoopFSM(Robot, telemetry);


        this.follower = follower;
        this.telemetry = telemetry;
        this.side = side;

        createPose();
        buildPaths();

        init();
    }
    public void createPose(){
        switch(side){
            case BUCKET:
                break;
            case OBSERVATION:
                startPose = new Pose(10.500, 60.500, Math.toRadians(180));
                specimen1Pose = new Pose(35.000, 71.500, Math.toRadians(180));
                specimen2Pose = new Pose(35,68,Math.toRadians(180));
                specimen3Pose = new Pose(35, 64.5,Math.toRadians(180));
                specimenControlPoint1Pose = new Pose(17, 46.5); // What is the direction on the robot?
                specimenControlPoint2Pose = new Pose(22, 64);
                curveControlPoint1Pose = new Pose(34.5, 33.5);
                curveControlPoint2Pose = new Pose(59, 41.5);
                longBack2Pose= new Pose(66, 25, Math.toRadians(0));
                longBack3Pose= new Pose(66, 14,Math.toRadians(0));
                shortBack1Pose = new Pose(26.5, 71.5, Math.toRadians(180));
                shift3Pose = new Pose(20, 14, Math.toRadians(0));
                shift2Pose = new Pose(20, 25,Math.toRadians(0)) ;
                pickup3Pose = new Pose(9.5, 14, Math.toRadians(0));
                pickup2Pose = new Pose(9.5, 25, Math.toRadians(0));
                break;
        }
    }
    public void buildPaths(){
        switch(side) {
            case BUCKET:
                break;
            case OBSERVATION:
                goal1 = new Path(new BezierCurve(new Point(startPose), new Point(specimen1Pose)));
                goal1.setConstantHeadingInterpolation(specimen1Pose.getHeading());

            moveCurve = follower.pathBuilder()
                        .addPath(new BezierLine(new Point(specimen1Pose), new Point(shortBack1Pose)))
                        .setConstantHeadingInterpolation(shortBack1Pose.getHeading())
                        .addPath(new BezierCurve(new Point(shortBack1Pose), new Point(curveControlPoint1Pose), new Point(curveControlPoint2Pose),  new Point(longBack2Pose)))
                        .setLinearHeadingInterpolation(shortBack1Pose.getHeading(), longBack2Pose.getHeading())
                        .build();


            push23 = follower.pathBuilder()
                    .addPath(new BezierLine(new Point(longBack2Pose), new Point(pickup2Pose)))
                    .setConstantHeadingInterpolation(pickup2Pose.getHeading())
                    .addPath(new BezierLine(new Point(pickup2Pose), new Point(longBack2Pose)))
                    .setConstantHeadingInterpolation(longBack2Pose.getHeading())
                    .addPath(new BezierLine(new Point(longBack2Pose), new Point(longBack3Pose)))
                    .setConstantHeadingInterpolation(longBack2Pose.getHeading())
                    .addPath(new BezierLine(new Point(longBack3Pose), new Point(pickup3Pose)))
                    .setConstantHeadingInterpolation(longBack3Pose.getHeading())
                    .build();




                goal2 = follower.pathBuilder()
                        .addPath(new BezierLine(new Point(pickup3Pose), new Point(shift3Pose)))
                    .setConstantHeadingInterpolation(pickup3Pose.getHeading())
                    .addPath(new BezierLine(new Point(shift3Pose), new Point(shift2Pose)))
                    .setConstantHeadingInterpolation(shift3Pose.getHeading())
                    .addPath(new BezierCurve(new Point(shift2Pose), new Point(specimenControlPoint1Pose), new Point(specimenControlPoint2Pose), new Point(specimen2Pose)))
                    .setLinearHeadingInterpolation(shift2Pose.getHeading(), specimen2Pose.getHeading())
                    .build();

             gather3 = follower.pathBuilder()
                     .addPath(new BezierCurve(new Point(specimen2Pose), new Point(specimenControlPoint2Pose), new Point(specimenControlPoint1Pose), new Point(shift2Pose)))
                     .setLinearHeadingInterpolation(specimen2Pose.getHeading(), shift2Pose.getHeading())
                     .addPath(new BezierLine(new Point(shift2Pose), new Point(pickup2Pose)))
                     .setConstantHeadingInterpolation(pickup2Pose.getHeading())
                     .build();


            goal3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Point(pickup2Pose), new Point(shift2Pose)))
                    .setConstantHeadingInterpolation((shift2Pose.getHeading()))
                    .addPath(new BezierCurve(new Point(shift2Pose), new Point(specimenControlPoint1Pose), new Point(specimenControlPoint2Pose), new Point(specimen3Pose)))
                    .setLinearHeadingInterpolation(shift2Pose.getHeading(), specimen3Pose.getHeading())
                    .build();



            score[1][0] = new Path(new BezierCurve(new Point(specimen1Pose), new Point(specimen1Pose.getX() + DISTANCE, specimen1Pose.getY())));
            score[1][0].setConstantHeadingInterpolation(specimen1Pose.getHeading());

            score[1][1] = new Path(new BezierCurve(new Point(specimen1Pose.getX() + DISTANCE, specimen1Pose.getY()), new Point(specimen1Pose)));
            score[1][1].setConstantHeadingInterpolation(specimen1Pose.getHeading());

            score[2][0] = new Path(new BezierCurve(new Point(specimen2Pose), new Point(specimen2Pose.getX() + DISTANCE, specimen2Pose.getY())));
            score[2][0].setConstantHeadingInterpolation(specimen2Pose.getHeading());

            score[2][1] = new Path(new BezierCurve(new Point(specimen2Pose.getX() + DISTANCE, specimen2Pose.getY()), new Point(specimen2Pose)));
            score[2][1].setConstantHeadingInterpolation(specimen2Pose.getHeading());

            score[3][0] = new Path(new BezierCurve(new Point(specimen3Pose), new Point(specimen3Pose.getX() + DISTANCE, specimen3Pose.getY())));
            score[3][0].setConstantHeadingInterpolation(specimen3Pose.getHeading());

            score[3][1] = new Path(new BezierCurve(new Point(specimen3Pose.getX() + DISTANCE, specimen3Pose.getY()), new Point(specimen3Pose)));
            score[3][1].setConstantHeadingInterpolation(specimen3Pose.getHeading());




            break;
        }



        moveCurve = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(9.484, 107.064),
                                new Point(34.440, 107.813),
                                new Point(13.227, 73.373),
                                new Point(39.182, 73.373)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();
        forwards = new Path(new BezierLine(new Point(0,0, Point.CARTESIAN), new Point(10,0, Point.CARTESIAN)));
        forwards.setConstantHeadingInterpolation(0);
        backwards = new Path(new BezierLine(new Point(10,0, Point.CARTESIAN), new Point(0,0, Point.CARTESIAN)));
        backwards.setConstantHeadingInterpolation(0);

    }
    public void init() {
        LiftFSM.initialize();
    }
    public void start(){

    }
    public void update(){
        follower.update();
        LiftFSM.update();
        ClawFSM.update();
        ScoopFSM.update();


        transfer(); 
    }
    public void transfer(){
        switch(transferState){
            case 1:
                actionBusy = true;
                LiftFSM.setState(liftFSM.LiftState.MID);
                setTransferState(2);
                break;
            case 2:
                if(LiftFSM.actionNotBusy()){
                    ClawFSM.setWristState(clawFSM.ClawWristState.UP);
                    transferTimer.resetTimer();
                    setTransferState(3);
                }
                break;
            case 3:
                if (transferTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(score[specimenNum][0], false);
                    setTransferState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()){
                    ClawFSM.setGrabState(clawFSM.ClawGrabState.OPEN);
                    transferTimer.resetTimer();
                    setTransferState(5);
            }
                break;
            case 5:
                if (transferTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(score[specimenNum][1], false);
                    setTransferState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()){
                    ClawFSM.setWristState(clawFSM.ClawWristState.DOWN);
                    transferTimer.resetTimer();
                    setTransferState(7);
                }
                break;
            case 7:
                if (transferTimer.getElapsedTimeSeconds() > 0.5) {
                    LiftFSM.setState(liftFSM.LiftState.ZERO);
                    setTransferState(8);
                }
                break;

            case 8:
                if(LiftFSM.actionNotBusy()){
                    actionBusy = false;
                    specimenNum = -1;
                    setTransferState(-1);

                }

        }
    }
    public void deposit(){
        switch(depositState){
            case 1:
                actionBusy = true;
                ScoopFSM.setState(scoopFSM.ScoopState.SCORE);
                depositTimer.resetTimer();
                setDepositState(2);
                break;
            case 2:
                if(depositTimer.getElapsedTimeSeconds() > 0.5){
                    ScoopFSM.setState(scoopFSM.ScoopState.WAIT);
                    depositTimer.resetTimer();
                    setDepositState(3);
                }
                break;
            case 3:
                if(depositTimer.getElapsedTimeSeconds() > 0.5){
                    actionBusy = false;
                    setDepositState(-1);
                }
        }
    }
    public void setTransferState(int x) {
        transferState = x;
        telemetry.addData("Transfer", x);
    }

    public void setDepositState(int x){
        depositState = x;
        telemetry.addData("Deposit", x);
    }

    public void startTransfer(int specimenNum) {
        if (actionNotBusy()) {
            setTransferState(1);
            this.specimenNum = specimenNum;
        }

    }
    public void startDesposit(){
        if (actionNotBusy()) {
            setDepositState(1);
        }
    }


    public boolean actionNotBusy() {
        return !actionBusy;
    }

    public boolean notBusy() {
        return (!follower.isBusy() && actionNotBusy());
    }

}