import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.ElapsedTime;
import static pedroPathing.ConfigFile.CONFIGkP;
import static pedroPathing.ConfigFile.CONFIGkI;
import static pedroPathing.ConfigFile.CONFIGkD;
import static pedroPathing.ConfigFile.loopTime;

import pedroPathing.constants.Constants;
import util.robot;
import java.util.function.Supplier;

/**
 * This is an example teleop that showcases movement and util.robot-centric driving.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 12/30/2024
 */

@TeleOp(name = "Teleop")
public class Teleop extends OpMode {
    private Follower follower;
    private robot R;
    private Gamepad currentGamepad1;
    private Gamepad previousGamepad1;
    private Gamepad currentGamepad2;
    private Gamepad previousGamepad2;
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;

    private final Pose startPose = new Pose(0, 0, 0);
    private double defaultSpeed = 1;
    private double highSpeed = 1;
    private double launcher = 0.5;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    double error;
    private double integralSum = 0;
    private double LOOP_TIME = loopTime;
    private double lastError = 0;
    private double kP, kD, kI;
    private double pidOutput = 0; // current motor power
    private ElapsedTime pidTimer = new ElapsedTime();
    private double targetVelocity = 1300;
    private boolean launcherOn = false;
    private double runPID(double target, double current, double currentPower) {
        error = target - current;

        integralSum += error * LOOP_TIME;
        double derivative = (error - lastError) / LOOP_TIME;
        double deltaPower = (kP * error) + (kI * integralSum) + (kD * derivative);

        lastError = error;

        return currentPower + deltaPower;
    }

    /**
     * This method is call once when init is played, it initializes the follower
     **/
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();
        currentGamepad2 = new Gamepad();
        previousGamepad2 = new Gamepad();
        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();

        R = new robot(hardwareMap);
        pidTimer.reset();
    }

    /**
     * This method is called continuously after Init while waiting to be started.
     **/
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     **/
    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    /**
     * This is the main loop of the opmode and runs continuously after play
     **/
    @Override
    public void loop() {

        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        /* Update Pedro to move the util.robot based on:
        - Forward/Backward Movement: -gamepad1.left_stick_y
        - Left/Right Movement: -gamepad1.left_stick_x
        - Turn Left/Right Movement: -gamepad1.right_stick_x
        - Robot-Centric Mode: true
        */
        follower.update();
        telemetryM.update();

        double currentVelocity = R.shooter.getVelocity();
        error = targetVelocity - currentVelocity;

        kP = CONFIGkP;
        kI = CONFIGkI;
        kD = CONFIGkD;

        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x * 0.8,
                    true // Robot Centric
            );

                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier * 0.8,
                    true // Robot Centric
            );
        }

        // ticks_per_second = power * (max_rpm / 60) * ticks_per_revolution

        if (gamepad1.a && !previousGamepad1.a) {
            launcher = 1300;
            launcherOn = !launcherOn;
        }
        else if (gamepad1.dpad_right && !previousGamepad1.dpad_right){
            launcher += 50;
        }
        else if (gamepad1.dpad_left && !previousGamepad1.dpad_left){
            launcher -= 50;
        }

        if (gamepad1.x && !previousGamepad1.x) {
            R.intakeLow.setPower(1);
            //R.shooter.setVelocity(powerToTicksPerSecond(1));
        }
        else if (gamepad1.y && !previousGamepad1.y){
            R.intakeLow.setPower(0);
            //R.shooter.setVelocity(powerToTicksPerSecond(0));
        }
        if (gamepad1.dpad_up && !previousGamepad1.dpad_up) {
            R.intakeHigh.setPower(1);
            //R.shooter.setVelocity(powerToTicksPerSecond(1));
        }
        else if (gamepad1.dpad_down && !previousGamepad1.dpad_down){
            R.intakeHigh.setPower(0);
            //R.shooter.setVelocity(powerToTicksPerSecond(0));
        }
        if (launcherOn) {
            targetVelocity = launcher; // ticks/sec
            if (pidTimer.seconds() >= LOOP_TIME) {
                pidOutput = runPID(targetVelocity, currentVelocity, pidOutput);
                pidOutput = Math.max(0.0, Math.min(1.0, pidOutput)); // clamp to [0,1]
                R.shooter.setPower(pidOutput);
                pidTimer.reset();
            }
        } else {
            R.shooter.setPower(0);
            targetVelocity = 0;
            pidOutput = 0;
            integralSum = 0;
            lastError = 0;
        }


        telemetry.addData("targetVelocity", targetVelocity);
        telemetry.addData("launchPower", R.shooter.getPower());
        telemetry.addData("launchVelo", R.shooter.getVelocity());
        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);
    }

    //private double powerToTicksPerSecond(double power) {
        //return power * ((double) 6000 / 60) * 28;
    }