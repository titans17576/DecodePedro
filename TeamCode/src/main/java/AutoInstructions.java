import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;

public class AutoInstructions extends opMode {
    private Follower follower;
    public int pathState = -1;
    public decodeAuto auto;
    public robot R;
    private Pose startPose;
    public Timer pathTimer = new Timer();

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
