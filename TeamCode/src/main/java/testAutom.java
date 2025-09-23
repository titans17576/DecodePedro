import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.Constants;
import util.robot;

@Autonomous(name="testAutom")
public class testAutom extends OpMode {
    private Follower follower;

    public int pathState = -1;
    public Auto auto;
    public robot R;
    private Pose startPose = new Pose(0,0);
    public Timer pathTimer = new Timer();


    @Override
    public void init() {
        R = new robot(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        auto = new Auto(R, telemetry, follower, Auto.Side.OBSERVATION);
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
                auto.follower.followPath(auto.forwards, false);
                setPathState(2);
                break;
            case 2:
                if(auto.notBusy()) {
                    auto.follower.followPath(auto.backwards, false);
                    setPathState(3);
                }
                break;
            case 3:
                if(auto.notBusy()){
                    setPathState(-1);
                }
                break;
        }
    }
    public void setPathState(int x) {
        pathState = x;
        pathTimer.resetTimer();
    }
}
