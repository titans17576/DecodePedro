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
    public enum Side{
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
        pickup3Pose, pickup3Control1Pose, end1Pose
    ;

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
    public void createPose(){
        switch(side){
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
    public void buildPaths(){
        switch(side) {
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
    public void init() {

    }
    public void start(){

    }
    public void update(){
        follower.update();

        park();
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