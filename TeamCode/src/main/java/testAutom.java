import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import util.robot;

@Autonomous(name="testAutom")
public class testAutom extends OpMode {
    private Follower follower;

    public int pathState = -1;
    public Auto auto;

    public robot R;
    private final Pose startPose = new Pose(9.483535528596187, 107.06412478336222);
    public Timer pathTimer = new Timer();


    @Override
    public void init() {
        R = new robot();
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        auto = new Auto(R, telemetry, follower, Auto.Side.BUCKET);
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
