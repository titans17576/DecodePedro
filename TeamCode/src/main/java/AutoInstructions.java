import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import pedroPathing.constants.Constants;
import util.robot;

@Autonomous(name="BlueAuto")

public class AutoInstructions extends OpMode {
    private Follower follower;
    public int pathState = -1;
    public decodeAuto auto;
    public robot R;
    private Pose startPose;
    public Timer pathTimer = new Timer();

    @Override
    public void init() {
        R = new robot(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        auto = new decodeAuto(R, telemetry, follower, decodeAuto.Side.BLUE);
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
                follower.setMaxPower(1);
                auto.intakeBalls(0);
                auto.follower.followPath(auto.scorePreload, true);
                setPathState(2);
                break;
            case 2:
                if (!auto.follower.isBusy()) {
                    auto.shoot(1);
                    setPathState(3);
                }
                break;
            case 3:
                auto.follower.followPath(auto.release1, true);
                setPathState(4);
                break;
            case 4:
                if (!auto.follower.isBusy()) {
                    auto.follower.followPath(auto.shoot1, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!auto.follower.isBusy()) {
                    auto.shoot(1);
                    setPathState(6);
                }
                break;
            case 6:
                auto.follower.followPath(auto.shoot2, true);
                setPathState(7);
                break;
            case 7:
                if (!auto.follower.isBusy()) {
                    auto.shoot(1);
                    setPathState(8);
                }
                break;
            case 8:
                auto.follower.followPath(auto.shoot3, true);
                setPathState(9);
                break;
            case 9:
                if (!auto.follower.isBusy()) {
                    auto.shoot(1);
                    setPathState(10);
                }
                break;
            case 10:
                auto.follower.followPath(auto.end, true);
                setPathState(11);
                break;
            case 11:
                if (!auto.follower.isBusy()) {
                    setPathState(-1);
                }
                break;
            default:
                break;
        }
    }

    public void setPathState(int x) {
        pathState = x;
        pathTimer.resetTimer();
    }
}
