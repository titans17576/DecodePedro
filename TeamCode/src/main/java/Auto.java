import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
    public Path forwards, backwards;


    public Pose startPose,
            specimen1Pose,specimen2Pose, specimen3Pose, specimen4Pose,specimen5Pose, preSpecPose,
            shortBack1Pose, longBack2Pose, longBack3Pose, longBack4Pose,
            shift2Pose, shift3Pose, shift4Pose,
            pickup2Pose, pickup3Pose, pickup3_5Pose, pickup4Pose,
            specimenControlPoint1Pose, specimenControlPoint2Pose,
            curveControlPoint1Pose, curveControlPoint2Pose,
            pushControlPointPose, push2ControlPointPose, push3ControlPointPose;


    public PathChain moveCurve, push23, gather3, gather4, gather5, goal2, goal3, goal4, goal5;
    public Path scorePreload, push1, park;

    public Path[][] score = new Path[5][2];
    public int DISTANCE = 1;

    public Auto(robot Robot, Telemetry telemetry, Follower follower, Side side) {



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
                startPose = new Pose(11, 61, Math.toRadians(0));
                specimen1Pose = new Pose(42, 75, Math.toRadians(0));
                specimen2Pose = new Pose(43,74,Math.toRadians(0));
                specimen3Pose = new Pose(43, 71,Math.toRadians(0));
                specimen4Pose = new Pose(43,68,Math.toRadians(0));
                specimen5Pose = new Pose(43, 65,Math.toRadians(0));
                preSpecPose = new Pose(30, 67.5,Math.toRadians(0));
                specimenControlPoint1Pose = new Pose(14.5, 70);
                specimenControlPoint2Pose = new Pose(28, 42);
                curveControlPoint1Pose = new Pose(30, 28);
                curveControlPoint2Pose = new Pose(59, 42);
                longBack2Pose = new Pose(64, 25, Math.toRadians(0));
                pushControlPointPose = new Pose(60,25, Math.toRadians(0));
                push2ControlPointPose = new Pose(60,18, Math.toRadians(0));
                push3ControlPointPose = new Pose(25,40,Math.toRadians(0));
                longBack3Pose= new Pose(60, 18,Math.toRadians(0));
                longBack4Pose = new Pose(60,11, Math.toRadians(0));
                shortBack1Pose = new Pose(26.5, 60, Math.toRadians(0));
                shift3Pose = new Pose(20, 18, Math.toRadians(0));
                shift2Pose = new Pose(20, 38,Math.toRadians(0));
                shift4Pose = new Pose(17, 20,Math.toRadians(0));
                pickup3Pose = new Pose(28, 18, Math.toRadians(0));
                pickup2Pose = new Pose(28, 25, Math.toRadians(0));
                pickup3_5Pose = new Pose(28, 11, Math.toRadians(0));
                pickup4Pose = new Pose(15, 38, Math.toRadians(0));
                break;
        }
    }
    public void buildPaths(){
        switch(side) {
            case BUCKET:
                break;
            case OBSERVATION:

            moveCurve = follower.pathBuilder()
                    .addPath(new BezierLine(specimen1Pose, shortBack1Pose))
                    .setLinearHeadingInterpolation(specimen1Pose.getHeading(),(shortBack1Pose).getHeading())
                    .addPath(new BezierCurve((shortBack1Pose), (curveControlPoint1Pose), (curveControlPoint2Pose), (longBack2Pose)))
                    .setLinearHeadingInterpolation(shortBack1Pose.getHeading(), longBack2Pose.getHeading())
                    .build();

            push23 = follower.pathBuilder()
                    .addPath(new BezierLine((longBack2Pose), (pickup2Pose)))
                    .setConstantHeadingInterpolation(pickup2Pose.getHeading())
                    .addPath(new BezierCurve((pickup2Pose), (pushControlPointPose), (longBack3Pose)))
                    .setConstantHeadingInterpolation(longBack3Pose.getHeading())
                    .addPath(new BezierLine((longBack3Pose), (pickup3Pose)))
                    .setConstantHeadingInterpolation(longBack3Pose.getHeading())
                    .addPath(new BezierCurve((pickup3Pose), (push2ControlPointPose), (longBack4Pose)))
                    .setConstantHeadingInterpolation(longBack3Pose.getHeading())
                    .addPath(new BezierLine((longBack4Pose), (pickup3_5Pose)))
                    .setConstantHeadingInterpolation(longBack3Pose.getHeading())
                    .addPath(new BezierCurve((pickup3_5Pose), (push3ControlPointPose), (pickup4Pose)))
                    .setConstantHeadingInterpolation(longBack3Pose.getHeading())
                    .build();

            goal2 = follower.pathBuilder()
                    .addPath(new BezierCurve((pickup4Pose), (specimenControlPoint2Pose), (specimenControlPoint1Pose), (specimen2Pose)))
                    .setConstantHeadingInterpolation((specimen2Pose.getHeading()))
                    .build();

            gather3 = follower.pathBuilder()
                    .addPath(new BezierCurve((specimen2Pose), (specimenControlPoint1Pose), (specimenControlPoint2Pose), (pickup4Pose)))
                    .setConstantHeadingInterpolation(pickup4Pose.getHeading())
                    .build();


            goal3 = follower.pathBuilder()
                    .addPath(new BezierCurve((pickup4Pose), (specimenControlPoint2Pose), (specimenControlPoint1Pose), (specimen3Pose)))
                    .setConstantHeadingInterpolation((specimen3Pose.getHeading()))
                    .build();

            gather4 = follower.pathBuilder()
                    .addPath(new BezierCurve((specimen3Pose), (specimenControlPoint1Pose), (specimenControlPoint2Pose), (pickup4Pose)))
                    .setConstantHeadingInterpolation(pickup4Pose.getHeading())
                    .build();

            goal4 = follower.pathBuilder()
                    .addPath(new BezierCurve((pickup4Pose), (specimenControlPoint2Pose), (specimenControlPoint1Pose), (specimen4Pose)))
                    .setConstantHeadingInterpolation((specimen4Pose.getHeading()))
                    .build();

            gather5 = follower.pathBuilder()
                    .addPath(new BezierCurve((specimen4Pose), (specimenControlPoint1Pose), (specimenControlPoint2Pose), (pickup4Pose)))
                    .setConstantHeadingInterpolation((specimen4Pose.getHeading()))
                    .build();

            goal5 = follower.pathBuilder()
                    .addPath(new BezierCurve((pickup4Pose), (specimenControlPoint2Pose), (specimenControlPoint1Pose), (specimen5Pose)))
                    .setConstantHeadingInterpolation((specimen5Pose.getHeading()))
                    .build();


            /*score[1][0] = new Path(new BezierCurve((specimen1Pose), (specimen1Pose.getX() + DISTANCE, specimen1Pose.getY())));
            score[1][0].setConstantHeadingInterpolation(specimen1Pose.getHeading());

            score[1][1] = new Path(new BezierCurve((specimen1Pose.getX() + DISTANCE, specimen1Pose.getY()), (specimen1Pose)));
            score[1][1].setConstantHeadingInterpolation(specimen1Pose.getHeading());

            score[2][0] = new Path(new BezierCurve((specimen2Pose), (specimen2Pose.getX() + DISTANCE, specimen2Pose.getY())));
            score[2][0].setConstantHeadingInterpolation(specimen2Pose.getHeading());

            score[2][1] = new Path(new BezierCurve((specimen2Pose.getX() + DISTANCE, specimen2Pose.getY()), (specimen2Pose)));
            score[2][1].setConstantHeadingInterpolation(specimen2Pose.getHeading());

            score[3][0] = new Path(new BezierCurve((specimen3Pose), (specimen3Pose.getX() + DISTANCE, specimen3Pose.getY())));
            score[3][0].setConstantHeadingInterpolation(specimen3Pose.getHeading());

            score[3][1] = new Path(new BezierCurve((specimen3Pose.getX() + DISTANCE, specimen3Pose.getY()), (specimen3Pose)));
            score[3][1].setConstantHeadingInterpolation(specimen3Pose.getHeading());*/




            break;
        }


        scorePreload = new Path(new BezierCurve((startPose), (specimen1Pose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), specimen1Pose.getHeading());
        push1 = new Path(new BezierCurve((shortBack1Pose), (curveControlPoint1Pose), (curveControlPoint2Pose),  (longBack2Pose)));
        push1.setLinearHeadingInterpolation(shortBack1Pose.getHeading(), longBack2Pose.getHeading());
        park = new Path(new BezierCurve((specimen4Pose), (specimenControlPoint1Pose), (specimenControlPoint2Pose), (shift4Pose)));
        park.setConstantHeadingInterpolation(specimen4Pose.getHeading());


    }
    public void init() {

    }
    public void start(){

    }
    public void update(){
        follower.update();
    }
    /*public void transfer(){
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
                depositTimer.resetTimer();
                setDepositState(2);
                break;
            case 2:
                if(depositTimer.getElapsedTimeSeconds() > 1.25){

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

    public void extendSweep(){
        switch(extendSweepState){
            case 1:
                actionBusy = true;
                IntakeFSM.setExtendoState(intakeFSM.ExtendoState.EXTEND);
                IntakeFSM.setArmState(intakeFSM.ArmState.GRAB);
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
                if(specScoreTimer.getElapsedTimeSeconds() > 0.3){
                    SpecimenFSM.setWristState(specimenFSM.ClawWristState.UP);
                    SpecimenFSM.setSpecArmState(specimenFSM.SpecArmState.HANG);
                    setScoreSpecState(3);
                }
                break;
            case 3:
                if(specScoreTimer.getElapsedTimeSeconds() > 0.7){
                    actionBusy = false;
                    setScoreSpecState(-1);
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
                if (postSpecScoreTimer.getElapsedTimeSeconds() > 0.2) {
                    actionBusy = false;
                    SpecimenFSM.setWristState(specimenFSM.ClawWristState.DOWN);
                    SpecimenFSM.setSpecArmState(specimenFSM.SpecArmState.GRAB);
                    SpecimenFSM.setLiftState(specimenFSM.LiftState.ZERO);
                    postSpecScoreTimer.resetTimer();
                    setPostSpecScoreState(3);
                }
                break;
            case 3:
                if(postSpecScoreTimer.getElapsedTimeSeconds() > 1) {
                    setPostSpecScoreState(-1);
                }
                break;
        }

    }
    public void postSpecScore2(){
        switch(postSpecScoreState2){
            case 1:
                actionBusy = true;
                SpecimenFSM.setGrabState(specimenFSM.ClawGrabState.OPEN);
                postSpecScoreTimer.resetTimer();
                setPostSpecScoreState(2);
                break;
            case 2:
                if (postSpecScoreTimer.getElapsedTimeSeconds() > 0.5) {
                    actionBusy = false;
                    SpecimenFSM.setWristState(specimenFSM.ClawWristState.MID);
                    SpecimenFSM.setSpecArmState(specimenFSM.SpecArmState.GRAB);
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

    }*/

    public void park(){
        switch(parkState){
            case 1:

                break;
            case 2:
                setParkState(-1);
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
    public void setPostSpecScoreState2(int x){
        postSpecScoreState2 = x;
        telemetry.addData("PostScoreSpec2", x);
    }

    public void setParkState(int x){
        parkState = x;
        telemetry.addData("Park", x);
    }
    public void setExtendSweepState(int x){
        extendSweepState = x;
        telemetry.addData("ExtendSweep", x);
    }
    public void setExtendRetractState(int x){
        extendRetractState = x;
        telemetry.addData("ExtendRetract", x);
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
    public void startPostSpecScore2(){
        if (actionNotBusy()){
            setPostSpecScoreState2(1);
        }
    }
    public void startExtendSweep(){
        if (actionNotBusy()){
            setExtendSweepState(1);
        }
    }
    public void startExtendRetract(){
        if (actionNotBusy()){
            setExtendRetractState(1);
        }
    }

    public void startPark(){
        setParkState(1);
    }


    public boolean actionNotBusy() {
        return !actionBusy;
    }

    public boolean notBusy() {
        return (!follower.isBusy() && actionNotBusy());
    }

}