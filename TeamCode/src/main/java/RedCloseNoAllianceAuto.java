import static pedroPathing.ConfigFile.CONFIGkD;
import static pedroPathing.ConfigFile.CONFIGkI;
import static pedroPathing.ConfigFile.CONFIGkP;
import static pedroPathing.ConfigFile.CONFIGkS;
import static pedroPathing.ConfigFile.CONFIGkV;
import static pedroPathing.ConfigFile.LOOPTIME;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.constants.Constants;
import util.robot;

@Autonomous(name="RedCloseNoAllianceAuto")

public class RedCloseNoAllianceAuto extends OpMode {
    private Follower follower;
    public int pathState = -1;
    public autoConfig auto;
    public robot R;
    private Pose startPose;
    public Timer pathTimer = new Timer();
    public Timer accelTimer = new Timer();
    public Timer delayTimer = new Timer();
    public Timer failsafeTimer = new Timer();
    public boolean actionBusy;
    private double kP, kI, kD, kV, kS;
    double error;

    private double integralSum = 0;
    private double lastError = 0;
    private double pidOutput = 0; // current motor power

    private ElapsedTime pidTimer = new ElapsedTime();

    private double LOOP_TIME = LOOPTIME;
    private double targetVelocity = 1140;
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
        auto = new autoConfig(R, telemetry, follower, autoConfig.Side.REDCLOSE);
        startPose = auto.startPose;
        follower.setStartingPose(startPose);
        kP = CONFIGkP;
        kI = CONFIGkI;
        kD = CONFIGkD;
        kV = CONFIGkV;
        kS = CONFIGkS;
    }

    @Override
    public void start() {
        auto.start();
        setPathState(1);
    }
    @Override
    public void loop() {
        /*telemetry.addData("State: ", pathState);
        telemetry.addData("Path Timer: ", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Current Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Shoot State", auto.shootState);
        telemetry.addData("Action Busy", auto.actionBusy);*/
        auto.update();
        pathUpdate();
        double currentVelocity = R.shooter.getVelocity();
        error = targetVelocity - currentVelocity;


        if (pidTimer.seconds() >= LOOP_TIME) {
            pidOutput = ((kV * targetVelocity) + (kP * (targetVelocity - R.shooter.getVelocity())) + kS);
            pidOutput = Math.max(0.0, Math.min(1.0, pidOutput)); // clamp to [0,1]
            R.shooter.setPower(pidOutput);
            R.shooter2.setPower(pidOutput);
            pidTimer.reset();
        } /*else {
            R.shooter.setPower(0);
            targetVelocity = 0;
            pidOutput = 0;
            integralSum = 0;
            lastError = 0;
        }*/

        /*telemetry.update();*/
    }
    public void pathUpdate() {
        switch (pathState) {
            case 1:
                auto.follower.followPath(auto.shootPreload, true);
                failsafeTimer.resetTimer();
                setPathState(2);
                break;
            case 2:
                if (auto.notBusy() || (failsafeTimer.getElapsedTimeSeconds() > 5)) {
                    accelTimer.resetTimer();
                    failsafeTimer.resetTimer();
                    setPathState(3);
                }
                break;
            case 3:
                if (((accelTimer.getElapsedTimeSeconds() > 0.1) && (auto.notBusy())) || (failsafeTimer.getElapsedTimeSeconds() > 5)) {
                    auto.startShoot();
                    failsafeTimer.resetTimer();
                    setPathState(4);
                }
                break;
            case 4:
                if (auto.notBusy() || (failsafeTimer.getElapsedTimeSeconds() > 5)) {
                    auto.startIntake();
                    auto.follower.followPath(auto.intake2, false);
                    failsafeTimer.resetTimer();
                    setPathState(5);
                }
                break;
            case 5:
                if (auto.notBusy() || (failsafeTimer.getElapsedTimeSeconds() > 5)) {
                    auto.stopIntake();
                    auto.follower.followPath(auto.shoot2, true);
                    failsafeTimer.resetTimer();
                    setPathState(6);
                }
                break;
            case 6:
                if (auto.notBusy() || (failsafeTimer.getElapsedTimeSeconds() > 5)) {
                    auto.startShoot();
                    failsafeTimer.resetTimer();
                    setPathState(7);
                }
                break;
            case 7:
                if (auto.notBusy() || (failsafeTimer.getElapsedTimeSeconds() > 5)) {
                    auto.startIntake();
                    auto.follower.followPath(auto.release1, true);
                    failsafeTimer.resetTimer();
                    setPathState(8);
                }
                break;
            case 8:
                if (auto.notBusy() || (failsafeTimer.getElapsedTimeSeconds() > 5)) {
                    delayTimer.resetTimer();
                    failsafeTimer.resetTimer();
                    setPathState(9);
                }
            case 9:
                if (((delayTimer.getElapsedTimeSeconds() > 1) && (auto.notBusy())) || (failsafeTimer.getElapsedTimeSeconds() > 5)) {
                    auto.follower.followPath(auto.shootGate, true);
                    delayTimer.resetTimer();
                    failsafeTimer.resetTimer();
                    setPathState(10);
                }
                break;
            case 10:
                if ((delayTimer.getElapsedTimeSeconds() > 0.5) || (failsafeTimer.getElapsedTimeSeconds() > 5)) {
                    auto.stopIntake();
                    failsafeTimer.resetTimer();
                    setPathState(11);
                }
                break;
            case 11:
                if (auto.notBusy() || (failsafeTimer.getElapsedTimeSeconds() > 5)) {
                    auto.startShoot();
                    failsafeTimer.resetTimer();
                    setPathState(12);
                }
                break;
            case 12:
                if (auto.notBusy() || (failsafeTimer.getElapsedTimeSeconds() > 5)) {
                    auto.startIntake();
                    auto.follower.followPath(auto.release1, true);
                    failsafeTimer.resetTimer();
                    setPathState(13);
                }
                break;
            case 13:
                if (auto.notBusy() || (failsafeTimer.getElapsedTimeSeconds() > 5)) {
                    delayTimer.resetTimer();
                    failsafeTimer.resetTimer();
                    setPathState(14);
                }
            case 14:
                if (((delayTimer.getElapsedTimeSeconds() > 1) && (auto.notBusy())) || (failsafeTimer.getElapsedTimeSeconds() > 5)) {
                    auto.follower.followPath(auto.shootGate, true);
                    delayTimer.resetTimer();
                    failsafeTimer.resetTimer();
                    setPathState(15);
                }
                break;
            case 15:
                if ((delayTimer.getElapsedTimeSeconds() > 0.5) || (failsafeTimer.getElapsedTimeSeconds() > 5)) {
                    auto.stopIntake();
                    failsafeTimer.resetTimer();
                    setPathState(16);
                }
                break;
            case 16:
                if (auto.notBusy() || (failsafeTimer.getElapsedTimeSeconds() > 5)) {
                    auto.startShoot();
                    failsafeTimer.resetTimer();
                    setPathState(17);
                }
                break;
            case 17:
                if (auto.notBusy() || (failsafeTimer.getElapsedTimeSeconds() > 5)) {
                    auto.startIntake();
                    auto.follower.followPath(auto.intake1, true);
                    failsafeTimer.resetTimer();
                    setPathState(18);
                }
                break;
            case 18:
                if (((auto.notBusy())) || (failsafeTimer.getElapsedTimeSeconds() > 5)) {
                    auto.follower.followPath(auto.shoot1, true);
                    delayTimer.resetTimer();
                    failsafeTimer.resetTimer();
                    setPathState(19);
                }
                break;
            case 19:
                if ((delayTimer.getElapsedTimeSeconds() > 0.5) || (failsafeTimer.getElapsedTimeSeconds() > 5)) {
                    auto.stopIntake();
                    failsafeTimer.resetTimer();
                    setPathState(20);
                }
                break;
            case 20:
                if (auto.notBusy() || (failsafeTimer.getElapsedTimeSeconds() > 5)) {
                    auto.startShoot();
                    failsafeTimer.resetTimer();
                    setPathState(21);
                }
                break;
            case 21:
                if (auto.notBusy() || (failsafeTimer.getElapsedTimeSeconds() > 5)) {
                    auto.startIntake();
                    auto.follower.followPath(auto.intake3, false);
                    failsafeTimer.resetTimer();
                    setPathState(22);
                }
                break;
            case 22:
                if (auto.notBusy() || (failsafeTimer.getElapsedTimeSeconds() > 5)) {
                    auto.stopIntake();
                    auto.follower.followPath(auto.shoot3, true);
                    failsafeTimer.resetTimer();
                    setPathState(23);
                }
            case 23:
                if (auto.notBusy() || (failsafeTimer.getElapsedTimeSeconds() > 5)) {
                    auto.startShoot();
                    failsafeTimer.resetTimer();
                    setPathState(24);
                }
                break;
            case 24:
                if (auto.notBusy() || (failsafeTimer.getElapsedTimeSeconds() > 5)) {
                    auto.follower.followPath(auto.end, true);
                    setPathState(25);
                }
                break;
            case 25:
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
