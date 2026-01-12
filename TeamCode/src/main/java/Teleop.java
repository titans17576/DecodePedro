import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.FtcDashboard;
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
import com.qualcomm.robotcore.util.ElapsedTime;

import static pedroPathing.ConfigFile.CONFIGkP;
import static pedroPathing.ConfigFile.CONFIGkV;
import static pedroPathing.ConfigFile.CONFIGkS;
import static pedroPathing.ConfigFile.LOOPTIME;

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
    private Supplier<PathChain> blueClose, redClose;

    private TelemetryManager telemetryM;
    private FtcDashboard dashboard;
    private intakeFSM IntakeFSM;
    private AutoAimer aimer;

    private final Pose startPose = new Pose(36, 72, Math.toRadians(180));
    private final ElapsedTime pidTimer = new ElapsedTime();
    private double launcher = 0.5;
    private boolean slowMode = true;
    private double slowModeMultiplier = 0.75;
    double error;
    private double kP, kV, kS, loopTime;

    private double targetVelocity = 0;
    private boolean launcherOn = false;

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

        blueClose = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, follower::getPose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(133), 0.8))
                .build();
        redClose = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, follower::getPose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(47), 0.8))
                .build();

        R = new robot(hardwareMap);
        aimer = new AutoAimer(R, telemetry);
        IntakeFSM = new intakeFSM(R, telemetry);
        pidTimer.reset();
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
        dashboard = FtcDashboard.getInstance(); //launcher tuning
    }

    /**
     * This method is called once at the start of the OpMode.
     **/
    @Override
    public void start() {
        follower.startTeleopDrive();
        kP = CONFIGkP;
        kV = CONFIGkV;
        kS = CONFIGkS;
        loopTime = LOOPTIME;
    }

    /**
     * Called once at the end of the OpMode
     */
    @Override
    public void stop() {
        aimer.stop();
    }

    /**
     * This is the main loop of the opmode and runs continuously after play
     **/
    @Override
    public void loop() {
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        follower.update();
        telemetryM.update();
        aimer.update();
        IntakeFSM.teleopUpdate(currentGamepad1, previousGamepad1);

        boolean aimerActive = gamepad1.right_trigger > 0.5;
        aimer.setActive(aimerActive);

        double currentVelocity = R.shooter.getVelocity();
        error = targetVelocity - currentVelocity;

        /* Update Pedro to move the util.robot based on:
        - Forward/Backward Movement: -gamepad1.left_stick_y
        - Left/Right Movement: -gamepad1.left_stick_x
        - Turn Left/Right Movement: -gamepad1.right_stick_x
        - Robot-Centric Mode: true
        */
        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            double turn = -gamepad1.right_stick_x * 0.8;

            if (aimerActive) {
                turn = aimer.getAverageMotorPower();
                telemetry.addData("hasTarget",  aimer.hasTarget());
                telemetry.addData("turn",  turn);
            }

            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    turn,
                    true // Robot Centric
            );

            //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    turn * (aimerActive ? 1 : slowModeMultiplier), // don't slow down auto aimer
                    true // Robot Centric
            );

        }

        /*if (gamepad1.left_stick_button && !previousGamepad1.left_stick_button) {
            follower.followPath(blueClose.get());
            automatedDrive = true;
        }
        if (gamepad1.right_stick_button && !previousGamepad1.right_stick_button) {
            follower.followPath(redClose.get());
            automatedDrive = true;
        }

        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.dpad_left && !previousGamepad1.dpad_left) || !follower.isBusy()) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }*/
        if (gamepad1.a && !previousGamepad1.a) {
            kV = CONFIGkV; //for tuning purposes
            kP = CONFIGkP; //was 0.000018
            launcher = 1200;
            launcherOn = !launcherOn;
            IntakeFSM.setGatekeepState(intakeFSM.GatekeepState.OFF);
        } else if (gamepad1.b && !previousGamepad1.b) {
            kV = CONFIGkV; //for tuning purposes
            kP = CONFIGkP; //was 0.000018
            launcher = 1500; /*far launch zone velocity*/
            launcherOn = !launcherOn;
            IntakeFSM.setGatekeepState(intakeFSM.GatekeepState.OFF);
        }
        if (gamepad1.dpad_right && !previousGamepad1.dpad_right) {
            targetVelocity += 20;
        } else if (gamepad1.dpad_left && !previousGamepad1.dpad_left) {
            targetVelocity -= 20;
        }


        if (launcherOn) {
            targetVelocity = launcher; // ticks/sec
            if (pidTimer.seconds() >= loopTime) {
                double pidOutput = ((kV * targetVelocity) + (kP * (targetVelocity - R.shooter.getVelocity())) + kS);
                pidOutput = Math.max(0.0, Math.min(1.0, pidOutput)); // clamp to [0,1]
                R.shooter.setPower(pidOutput);
                R.shooter2.setPower(pidOutput);
                pidTimer.reset();
            }
        } else {
            R.shooter.setPower(0);
            R.shooter2.setPower(0);
            //targetVelocity = 0;
            //pidOutput = 0;
        }

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Launcher Velocity", R.shooter.getVelocity());
        packet.put("Target Velocity", targetVelocity);
        dashboard.sendTelemetryPacket(packet); // launcher tuning

        telemetry.addLine();
        telemetry.addData("targetVelocity", targetVelocity);
        telemetry.addData("launchPower", R.shooter.getPower());
        telemetry.addData("launchVelo", R.shooter.getVelocity());
        telemetry.addData("transferVelocity", R.intakeHigh.getVelocity());
        telemetry.addData("automatedDrive", automatedDrive);
        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);
    }
}