import static pedroPathing.ConfigFile.CONFIGkD;
import static pedroPathing.ConfigFile.CONFIGkI;
import static pedroPathing.ConfigFile.CONFIGkP;
import static pedroPathing.ConfigFile.loopTime;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.constants.Constants;
import util.robot;

@Autonomous(name="BlueCloseAuto")

public class BlueCloseAuto extends OpMode {
    private Follower follower;
    public int pathState = -1;
    public decodeAuto auto;
    public robot R;
    private Pose startPose;
    public Timer pathTimer = new Timer();
    public Timer accelTimer = new Timer();
    public boolean actionBusy;
    private double kP, kI, kD;
    double error;

    private double integralSum = 0;
    private double lastError = 0;
    private double pidOutput = 0; // current motor power

    private ElapsedTime pidTimer = new ElapsedTime();

    private double targetVelocity = 1300;
    private double LOOP_TIME = loopTime;
    private boolean launcherOn = false;
    private double runPID(double target, double current, double currentPower) {
        error = target - current;

        integralSum += error * LOOP_TIME;
        double derivative = (error - lastError) / LOOP_TIME;
        double deltaPower = (kP * error) + (kI * integralSum) + (kD * derivative);

        lastError = error;

        return currentPower + deltaPower;
    }

    @Override
    public void init() {
        R = new robot(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        auto = new decodeAuto(R, telemetry, follower, decodeAuto.Side.BLUECLOSE);
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
        telemetry.addData("Current Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Shoot State", auto.shootState);
        telemetry.addData("Action Busy", auto.actionBusy);
        auto.update();
        pathUpdate();
        double currentVelocity = R.shooter.getVelocity();
        error = targetVelocity - currentVelocity;

        kP = CONFIGkP;
        kI = CONFIGkI;
        kD = CONFIGkD;

        if (pidTimer.seconds() >= LOOP_TIME) {
            pidOutput = runPID(targetVelocity, currentVelocity, pidOutput);
            pidOutput = Math.max(0.0, Math.min(1.0, pidOutput)); // clamp to [0,1]
            R.shooter.setPower(pidOutput);
            pidTimer.reset();
        }

        telemetry.update();
    }
    public void pathUpdate() {
        switch (pathState) {
            case 1:
                follower.setMaxPower(0.8);
                auto.follower.followPath(auto.shootPreload, true);
                setPathState(2);
                break;
            case 2:
                if (auto.notBusy()) {
                    accelTimer.resetTimer();
                    setPathState(3);
                }
                break;
            case 3:
                if ((accelTimer.getElapsedTimeSeconds() > 2) && (auto.notBusy())) {
                    auto.startShoot();
                    //skip gate open
                    setPathState(5);
                }
                break;
            case 4:
                if (auto.notBusy()) {
                    auto.follower.followPath(auto.release1, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (auto.notBusy()) {
                    auto.startIntake();
                    auto.follower.followPath(auto.shoot1, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (auto.notBusy()) {
                    auto.startShoot();
                    setPathState(7);
                }
                break;
            case 7:
                if (auto.notBusy()) {
                    auto.startIntake();
                    auto.follower.followPath(auto.shoot2, true);
                    setPathState(8);
                }
                break;
            case 8:
                if (auto.notBusy()) {
                    auto.startShoot();
                    setPathState(9);
                }
                break;
            case 9:
                if (auto.notBusy()) {
                    auto.startIntake();
                    auto.follower.followPath(auto.shoot3, true);
                    setPathState(10);
                }
                break;
            case 10:
                if (auto.notBusy()) {
                    auto.startShoot();
                    setPathState(11);
                }
                break;
            case 11:
                if (auto.notBusy()) {
                    auto.follower.followPath(auto.end, true);
                    setPathState(12);
                }
                break;
            case 12:
                if (auto.notBusy()) {
                    setPathState(-1);
                }
        }
    }

    public void setPathState(int x) {
        pathState = x;
        pathTimer.resetTimer();
    }
}
