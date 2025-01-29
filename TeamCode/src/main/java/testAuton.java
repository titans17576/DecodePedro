import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import util.robot;

@Autonomous(name="testAuton")
public class testAuton extends OpMode {
    private Follower follower;

    public int pathState = -1;
    public Auto auto;
    public robot R;
    private Pose startPose = new Pose(10.500, 71.500, Math.toRadians(180));

    public Timer pathTimer = new Timer();


    @Override
    public void init() {
        R = new robot(hardwareMap);
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
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
                auto.follower.followPath(auto.scorePreload, false);
                setPathState(2);
                break;
            case 2:
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
