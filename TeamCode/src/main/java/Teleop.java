import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import static pedroPathing.ConfigFile.CONFIGHighkP;
import static pedroPathing.ConfigFile.CONFIGHighkI;
import static pedroPathing.ConfigFile.CONFIGHighkD;
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
    private boolean slowMode = true;
    private double slowModeMultiplier = 0.75;
    private double shooterPIDOutput = 0; // current motor power
    private double targetVelocity = 1300;
    private boolean launcherOn = false;
    private boolean intakeHighOn = false;
    private boolean intakeLowOn = false;

    private final PIDController shooterPID = new PIDController(CONFIGkP, CONFIGkI, CONFIGkD, loopTime);

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

        shooterPID.pidTimer.reset();

        R.shooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        R.shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        R.intakeHigh.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        R.intakeHigh.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
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

        double shooter_currentVelocity = R.shooter.getVelocity();
        shooterPID.error = targetVelocity - shooter_currentVelocity;

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

        /*if (gamepad2.dpad_left && !previousGamepad2.dpad_left) {
            slowModeMultiplier -= 0.25;
        }
        if (gamepad2.dpad_right && !previousGamepad2.dpad_right) {
            slowModeMultiplier += 0.25;
        }
        if (gamepad2.left_bumper && !previousGamepad2.left_bumper) {
            slowMode = !slowMode;
        }*/
        if (gamepad1.a && !previousGamepad1.a) {
            launcher = 1300;
            launcherOn = !launcherOn;
        }
        else if (gamepad1.b && !previousGamepad1.b) {
            launcher = 1550; /*far launch zone velocity*/
            launcherOn = !launcherOn;
        }
        else if (gamepad1.dpad_right && !previousGamepad1.dpad_right){
            launcher += 50;
        }
        else if (gamepad1.dpad_left && !previousGamepad1.dpad_left){
            launcher -= 50;
        }

        if (gamepad1.x && !previousGamepad1.x) { // TOGGLE LOW WITH X
            intakeLowOn = !intakeLowOn;
            R.intakeLow.setPower(intakeLowOn ? 1 : 0);
        }

        if (gamepad1.y) {
            intakeHighOn = true;
            R.intakeHigh.setPower(1);
        } else {
            intakeHighOn = false;
            R.intakeHigh.setPower(0);
        }

        if (gamepad1.dpad_up && !previousGamepad1.dpad_up) {
            R.intakeHigh.setPower(1);
            intakeHighOn = true;
        }
        else if (gamepad1.dpad_down && !previousGamepad1.dpad_down){
            intakeHighOn = false;
            R.intakeHigh.setPower(0);
        }

        if (gamepad1.left_bumper && !previousGamepad1.left_bumper){
            R.gatekeep.setPosition(0.4);
        } else if (gamepad1.right_bumper && !previousGamepad1.right_bumper){
            R.gatekeep.setPosition(0.3);
        }


        if (launcherOn) {
            targetVelocity = launcher; // ticks/sec
            if (shooterPID.pidTimer.seconds() >= shooterPID.loopTime) {
                shooterPIDOutput = shooterPID.runPID(targetVelocity, shooter_currentVelocity, shooterPIDOutput);
                shooterPIDOutput = Math.max(0.0, Math.min(1.0, shooterPIDOutput)); // clamp to [0,1]
                R.shooter.setPower(shooterPIDOutput);
                shooterPID.pidTimer.reset();
            }
        } else {
            R.shooter.setPower(0);
            targetVelocity = 0;
            shooterPIDOutput = 0;
            shooterPID.integralSum = 0;
            shooterPID.lastError = 0;
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