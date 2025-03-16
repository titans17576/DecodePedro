import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import util.robot;

@Autonomous(name="4_Spec")

public class ConsistentSpec extends OpMode {
    private Follower follower;
    public int pathState = -1;
    public Auto auto;
    public robot R;
    private Pose startPose;
    public Timer pathTimer = new Timer();


    @Override
    public void init() {
        R = new robot(hardwareMap);
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        auto = new Auto(R, telemetry, follower, Auto.Side.OBSERVATION);
        startPose = auto.startPose;
        follower.setStartingPose(startPose);
    }

    @Override
    public void start() {
        auto.start();
        setPathState(1);
    }

    @Override
    public void loop() {
        telemetry.addData("State: ", pathState);
        telemetry.addData("Path Timer: ", pathTimer.getElapsedTimeSeconds());
        auto.update();
        pathUpdate();

        telemetry.update();
    }

    public void pathUpdate() {
        switch (pathState) {
            case 1:
                follower.setMaxPower(0.9);
                auto.startSpecScore();
                setPathState(2);
                break;
            case 2:
                if(auto.notBusy()) {
                    auto.follower.followPath(auto.scorePreload, false);
                    setPathState(3);
                }
                break;
            case 3:
                auto.startPark();
                if(auto.notBusy()) {
                    auto.startPostSpecScore();
                    setPathState(4);
                }
                break;
            case 4:
                if(auto.notBusy()) {
                    follower.setMaxPower(0.8);
                    auto.follower.followPath(auto.moveCurve, true);
                    setPathState(5);
                }
                break;
            case 5:
                if(auto.notBusy()) {
                    R.liftMotor.setPower(0);
                    R.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    follower.setMaxPower(0.75);
                    auto.follower.followPath(auto.push23, true);
                    setPathState(6);
                }
                break;
            case 6:
                if(auto.notBusy()) {
                    auto.startSpecScore();
                    setPathState(7);
                }
                break;
            case 7:
                if(auto.notBusy()) {
                    follower.setMaxPower(0.9);
                    auto.follower.followPath(auto.goal2, true);
                    setPathState(8);
                }
                break;
            case 8:
                if(auto.notBusy()) {
                    auto.startPostSpecScore();
                    setPathState(9);
                }
                break;
            case 9:
                if(auto.notBusy()) {
                    follower.setMaxPower(0.8);
                    auto.follower.followPath(auto.gather3, true);
                    setPathState(10);
                }
                break;
            case 10:
                if(auto.notBusy()) {
                    R.liftMotor.setPower(0);
                    R.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    auto.startSpecScore();
                    setPathState(11);
                }
                break;
            case 11:
                if(auto.notBusy()) {
                    follower.setMaxPower(0.9);
                    auto.follower.followPath(auto.goal3, true);
                    setPathState(12);
                }
                break;
            case 12:
                if(auto.notBusy()) {
                    auto.startPostSpecScore();
                    setPathState(13);
                }
                break;
            case 13:
                if(auto.notBusy()) {
                    follower.setMaxPower(0.8);
                    auto.follower.followPath(auto.gather4, true);
                    setPathState(14);
                }
                break;
            case 14:
                if(auto.notBusy()) {
                    R.liftMotor.setPower(0);
                    R.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    auto.startSpecScore();
                    setPathState(15);
                }
                break;
            case 15:
                if(auto.notBusy()) {
                    follower.setMaxPower(0.9);
                    auto.follower.followPath(auto.goal4, true);
                    setPathState(16);
                }
                break;
            case 16:
                if(auto.notBusy()) {
                    auto.startPostSpecScore();
                    setPathState(17);
                }
                break;
            case 17:
                if(auto.notBusy()) {
                    auto.follower.followPath(auto.park, true);
                    setPathState(18);
                }
                break;
            case 18:
                if(auto.notBusy()) {
                    R.liftMotor.setPower(0);
                    R.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    setPathState(19);
                }
                break;
            case 19:
                if(auto.notBusy()){
                    setPathState(-1);
                }
        }
    }
    public void setPathState(int x) {
        pathState = x;
        pathTimer.resetTimer();
    }
}
