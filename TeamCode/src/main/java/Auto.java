import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
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

    public specimenFSM SpecimenFSM;
    public scoopFSM ScoopFSM;

    private Side side;
    public Timer transferTimer = new Timer();
    public Timer specScoreTimer = new Timer();
    public Timer depositTimer = new Timer();
    public Timer postSpecScoreTimer = new Timer();
    public int transferState = -1, specimenNum = -1;
    public int depositState = -1;
    public int scoreSpecState = -1;
    public int postSpecScoreState = -1;
    public int fakeTransferState = -1;
    public Path forwards, backwards;


    public Pose startPose,
            specimen1Pose,specimen2Pose, specimen3Pose, specimen4Pose,
            shortBack1Pose, longBack2Pose, longBack2_5Pose, longBack3Pose, longBack4Pose,
            shift2Pose, shift3Pose, shift4Pose,
            pickup2Pose, pickup3Pose, pickup4Pose,
            specimenControlPoint1Pose, specimenControlPoint2Pose,
            curveControlPoint1Pose, curveControlPoint2Pose;


    public PathChain moveCurve, push23, gather3, gather4, goal2, goal3, goal4;
    public Path scorePreload, push1;

    public Path[][] score = new Path[5][2];
    public int DISTANCE = 1;

    public Auto(robot Robot, Telemetry telemetry, Follower follower, Side side) {
        SpecimenFSM = new specimenFSM(Robot, telemetry);
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
                startPose = new Pose(10.5, 71.5, Math.toRadians(180));
                specimen1Pose = new Pose(35, 71.5, Math.toRadians(180));
                specimen2Pose = new Pose(37.5,69.5,Math.toRadians(180));
                specimen3Pose = new Pose(38, 67.5,Math.toRadians(180));
                specimen4Pose = new Pose(38, 66.5,Math.toRadians(180));
                specimenControlPoint1Pose = new Pose(17, 46.5); // What is the direction on the robot?
                specimenControlPoint2Pose = new Pose(22, 64);
                curveControlPoint1Pose = new Pose(34.5, 33.5);
                curveControlPoint2Pose = new Pose(59, 41.5);
                longBack2Pose= new Pose(66, 25, Math.toRadians(0));
                longBack2_5Pose = new Pose(62, 25, Math.toRadians(0));
                longBack3Pose= new Pose(62, 18,Math.toRadians(0));
                shortBack1Pose = new Pose(26.5, 60, Math.toRadians(180));
                shift3Pose = new Pose(20, 18, Math.toRadians(0));
                shift2Pose = new Pose(26, 25,Math.toRadians(0)) ;
                pickup3Pose = new Pose(15, 18, Math.toRadians(0));
                pickup2Pose = new Pose(28, 25, Math.toRadians(0));
                pickup4Pose = new Pose(15, 25, Math.toRadians(0));
                break;
        }
    }
    public void buildPaths(){
        switch(side) {
            case BUCKET:
                break;
            case OBSERVATION:

            moveCurve = follower.pathBuilder()
                    .addPath(new BezierLine(new Point(specimen1Pose), new Point(shortBack1Pose)))
                    .setLinearHeadingInterpolation(specimen1Pose.getHeading(), shortBack1Pose.getHeading())
                    .addPath(new BezierCurve(new Point(shortBack1Pose), new Point(curveControlPoint1Pose), new Point(curveControlPoint2Pose),  new Point(longBack2Pose)))
                    .setLinearHeadingInterpolation(shortBack1Pose.getHeading(), longBack2Pose.getHeading())
                    .build();

            push23 = follower.pathBuilder()
                    .addPath(new BezierLine(new Point(longBack2Pose), new Point(pickup2Pose)))
                    .setConstantHeadingInterpolation(pickup2Pose.getHeading())
                    .addPath(new BezierLine(new Point(pickup2Pose), new Point(longBack2_5Pose)))
                    .setConstantHeadingInterpolation(longBack2_5Pose.getHeading())
                    .addPath(new BezierLine(new Point(longBack2_5Pose), new Point(longBack3Pose)))
                    .setConstantHeadingInterpolation(longBack2_5Pose.getHeading())
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
                    .addPath(new BezierLine(new Point(shift2Pose), new Point(pickup4Pose)))
                    .setConstantHeadingInterpolation(pickup4Pose.getHeading())
                    .build();


            goal3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Point(pickup4Pose), new Point(shift2Pose)))
                    .setConstantHeadingInterpolation((shift2Pose.getHeading()))
                    .addPath(new BezierCurve(new Point(shift2Pose), new Point(specimenControlPoint1Pose), new Point(specimenControlPoint2Pose), new Point(specimen3Pose)))
                    .setLinearHeadingInterpolation(shift2Pose.getHeading(), specimen3Pose.getHeading())
                    .build();

            gather4 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Point(specimen3Pose), new Point(specimenControlPoint2Pose), new Point(specimenControlPoint1Pose), new Point(shift2Pose)))
                    .setLinearHeadingInterpolation(specimen3Pose.getHeading(), shift2Pose.getHeading())
                    .addPath(new BezierLine(new Point(shift2Pose), new Point(pickup4Pose)))
                    .setConstantHeadingInterpolation(pickup4Pose.getHeading())
                    .build();

            goal4 = follower.pathBuilder()
                    .addPath(new BezierLine(new Point(pickup4Pose), new Point(shift2Pose)))
                    .setConstantHeadingInterpolation((shift2Pose.getHeading()))
                    .addPath(new BezierCurve(new Point(shift2Pose), new Point(specimenControlPoint1Pose), new Point(specimenControlPoint2Pose), new Point(specimen4Pose)))
                    .setLinearHeadingInterpolation(shift2Pose.getHeading(), specimen4Pose.getHeading())
                    .build();


            /*score[1][0] = new Path(new BezierCurve(new Point(specimen1Pose), new Point(specimen1Pose.getX() + DISTANCE, specimen1Pose.getY())));
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
            score[3][1].setConstantHeadingInterpolation(specimen3Pose.getHeading());*/




            break;
        }


        scorePreload = new Path(new BezierCurve(new Point(startPose), new Point(specimen1Pose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), specimen1Pose.getHeading());
        push1 = new Path(new BezierCurve(new Point(shortBack1Pose), new Point(curveControlPoint1Pose), new Point(curveControlPoint2Pose),  new Point(longBack2Pose)));
        push1.setLinearHeadingInterpolation(shortBack1Pose.getHeading(), longBack2Pose.getHeading());

        forwards = new Path(new BezierLine(new Point(0,0, Point.CARTESIAN), new Point(10,0, Point.CARTESIAN)));
        forwards.setConstantHeadingInterpolation(0);
        backwards = new Path(new BezierLine(new Point(10,0, Point.CARTESIAN), new Point(0,0, Point.CARTESIAN)));
        backwards.setConstantHeadingInterpolation(0);

    }
    public void init() {

    }
    public void start(){

    }
    public void update(){
        follower.update();
        SpecimenFSM.update();
        ScoopFSM.update();


        transfer();
        fakeTransfer();
        scoreSpec();
        postSpecScore();
    }
    public void transfer(){
        switch(transferState){
            case 1:
                actionBusy = true;
                SpecimenFSM.setLiftState(specimenFSM.LiftState.MID);
                setTransferState(2);
                break;
            case 2:
                if(SpecimenFSM.actionNotBusy()){
                    SpecimenFSM.setWristState(specimenFSM.ClawWristState.UP);
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
                    SpecimenFSM.setGrabState(specimenFSM.ClawGrabState.OPEN);
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
                    SpecimenFSM.setWristState(specimenFSM.ClawWristState.DOWN);
                    transferTimer.resetTimer();
                    setTransferState(7);
                }
                break;
            case 7:
                if (transferTimer.getElapsedTimeSeconds() > 0.5) {
                    SpecimenFSM.setLiftState(specimenFSM.LiftState.ZERO);
                    setTransferState(8);
                }
                break;

            case 8:
                if(SpecimenFSM.actionNotBusy()){
                    actionBusy = false;
                    specimenNum = -1;
                    setTransferState(-1);

                }
                break;
        }
    }

    public void fakeTransfer(){
        switch(fakeTransferState){
            case 1:
                actionBusy = true;
                SpecimenFSM.setLiftState(specimenFSM.LiftState.MID);
                setFakeTransferState(2);
                break;
            case 2:
                if(SpecimenFSM.actionNotBusy()){
                    SpecimenFSM.setWristState(specimenFSM.ClawWristState.UP);
                    transferTimer.resetTimer();
                    setFakeTransferState(3);
                }
                break;
            case 3:
                if (transferTimer.getElapsedTimeSeconds() > 0.5) {
                    SpecimenFSM.setGrabState(specimenFSM.ClawGrabState.OPEN);
                    transferTimer.resetTimer();
                    setFakeTransferState(4);
                }
                break;
            case 4:
                if (transferTimer.getElapsedTimeSeconds() > 0.5) {
                    SpecimenFSM.setWristState(specimenFSM.ClawWristState.DOWN);
                    transferTimer.resetTimer();
                    setFakeTransferState(5);
                }
                break;
            case 5:
                if (transferTimer.getElapsedTimeSeconds() > 0.5) {
                    SpecimenFSM.setLiftState(specimenFSM.LiftState.ZERO);
                    setFakeTransferState(6);
                }
                break;

            case 6:
                if(SpecimenFSM.actionNotBusy()){
                    actionBusy = false;
                    specimenNum = -1;
                    setFakeTransferState(-1);

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
                if(depositTimer.getElapsedTimeSeconds() > 1.25){
                    ScoopFSM.setState(scoopFSM.ScoopState.WAIT);
                    depositTimer.resetTimer();
                    setDepositState(3);
                }
                break;
            case 3:
                if(depositTimer.getElapsedTimeSeconds() > 4){
                    actionBusy = false;
                    setDepositState(-1);
                }
        }
    }

    public void scoreSpec(){
        switch(scoreSpecState){
            case 1:
                actionBusy = true;
                SpecimenFSM.setGrabState(specimenFSM.ClawGrabState.CLOSED);
                SpecimenFSM.setLiftState(specimenFSM.LiftState.MID);
                specScoreTimer.resetTimer();
                setScoreSpecState(2);
                break;
            case 2:
                if(specScoreTimer.getElapsedTimeSeconds() > 1.25){
                    actionBusy = false;
                    SpecimenFSM.setWristState(specimenFSM.ClawWristState.UP);
                    setScoreSpecState(3);
                }
                break;
            case 3:
                if(specScoreTimer.getElapsedTimeSeconds() > 4){
                    setDepositState(-1);
                }
        }
    }

    public void postSpecScore(){
        switch(postSpecScoreState){
            case 1:
                actionBusy = true;
                SpecimenFSM.setGrabState(specimenFSM.ClawGrabState.OPEN);
                postSpecScoreTimer.resetTimer();
                setPostSpecScoreState(2);
                break;
            case 2:
                if (postSpecScoreTimer.getElapsedTimeSeconds() > 0.5) {
                    actionBusy = false;
                    SpecimenFSM.setWristState(specimenFSM.ClawWristState.DOWN);
                    SpecimenFSM.setLiftState(specimenFSM.LiftState.ZERO);
                    postSpecScoreTimer.resetTimer();
                    setPostSpecScoreState(3);
                }
                break;
            case 3:
                if(postSpecScoreTimer.getElapsedTimeSeconds() > 1.5) {
                    setPostSpecScoreState(-1);
                }
                break;
        }

    }
    public void setTransferState(int x) {
        transferState = x;
        telemetry.addData("Transfer", x);
    }

    public void setFakeTransferState(int x) {
        transferState = x;
        telemetry.addData("Transfer", x);
    }

    public void setDepositState(int x){
        depositState = x;
        telemetry.addData("Deposit", x);
    }
    public void setScoreSpecState(int x){
        scoreSpecState = x;
        telemetry.addData("ScoreSpec", x);
    }

    public void setPostSpecScoreState(int x){
        postSpecScoreState = x;
        telemetry.addData("PostScoreSpec", x);
    }

    public void startTransfer(int specimenNum) {
        if (actionNotBusy()) {
            setTransferState(1);
            this.specimenNum = specimenNum;
        }

    }
    public void startDeposit(){
        if (actionNotBusy()) {
            setDepositState(1);
        }
    }
    public void startSpecScore(){
        if (actionNotBusy()) {
            setScoreSpecState(1);
        }
    }
    
    public void startPostSpecScore(){
        if (actionNotBusy()){
            setPostSpecScoreState(1);
        }
    }


    public boolean actionNotBusy() {
        return !actionBusy;
    }

    public boolean notBusy() {
        return (!follower.isBusy() && actionNotBusy());
    }

}