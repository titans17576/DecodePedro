import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.HeadingInterpolator;
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

@TeleOp(name = "TeleopBlue")
public class TeleopBlue extends OpMode {
    private robot R;
    private Follower follower;
    private VisionController localizer;
    private Gamepad currentGamepad1;
    private Gamepad previousGamepad1;
    public Timer motifTimer = new Timer();

    private boolean automatedDrive;
    private Supplier<PathChain> blueClose, redClose;

    private TelemetryManager telemetryM;
    private FtcDashboard dashboard;
    private intakeFSM IntakeFSM;
    private double angleTowardsBlueGoal = 0;
    private double angleTowardsRedGoal = 0;
    private double distanceFromBlueGoal = 0;
    public double lookupValue = 1160;

    private final Pose end1Pose = new Pose(52, 104, Math.toRadians(125));
    private final Pose relocalizeBluePose = new Pose(63, 7, Math.toRadians(180));
    private final Pose relocalizeRedPose = new Pose(81, 7, Math.toRadians(0));
    private final Pose shoot1Pose = new Pose(48, 96, Math.toRadians(133));
    private Pose slightOffsetBlue = new Pose(52,104, angleTowardsBlueGoal);
    private Pose slightOffsetRed = new Pose(52,104, angleTowardsRedGoal);
    private final ElapsedTime pidTimer = new ElapsedTime();
    private double launcher = 0.5;
    private boolean slowMode = true;
    private boolean endgame = false;
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
        follower.setStartingPose(end1Pose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();

        R = new robot(hardwareMap);
        localizer = new VisionController(R, follower, telemetry);
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
        motifTimer.resetTimer();
    }

    /**
     * Called once at the end of the OpMode
     */
    @Override
    public void stop() {
        localizer.stop();
    }

    /**
     * This is the main loop of the opmode and runs continuously after play
     **/
    @Override
    public void loop() {
        currentGamepad1.copy(gamepad1);

        follower.update();
        telemetryM.update();

        IntakeFSM.teleopUpdate(currentGamepad1, previousGamepad1);

        double xFromBlueGoal = ((Math.abs(follower.getPose().getX()))-10);
        double xFromRedGoal = (135.5-(Math.abs(follower.getPose().getX())));
        double yFromGoal = (141.5-Math.abs(follower.getPose().getY()));
        double distanceFromBlueGoal = (Math.sqrt((xFromBlueGoal*xFromBlueGoal)+(yFromGoal * yFromGoal)));
        double distanceFromRedGoal = (Math.sqrt((xFromRedGoal*xFromRedGoal)+(yFromGoal * yFromGoal)));

        double angleTowardsBlueGoal = (Math.PI) - Math.atan2(yFromGoal,xFromBlueGoal);
        double angleTowardsRedGoal = Math.atan2(yFromGoal,xFromRedGoal);

        /*if (distanceFromBlueGoal >= 96) { //need to make toggle? was 96
            IntakeFSM.highIntakeOn_velocity = 1100; //slower transfer speed means less bounce outs due to artifact contact
        } else if (distanceFromBlueGoal < 96){
            IntakeFSM.highIntakeOn_velocity = 1600;
        }*/

        if (distanceFromBlueGoal <= 60) {
            lookupValue = 1160;
            kP = 0.007;
        } else if ((distanceFromBlueGoal > 60) && (distanceFromBlueGoal <= 72)) {
            lookupValue = 1190;
            kP = 0.007;
        } else if ((distanceFromBlueGoal > 72) && (distanceFromBlueGoal <= 84)) {
            lookupValue = 1220;
            kP = 0.007;
        } else if ((distanceFromBlueGoal > 84) && (distanceFromBlueGoal <= 96)) {
            lookupValue = 1240;
            kP = 0.007;
        } else if ((distanceFromBlueGoal > 96) && (distanceFromBlueGoal <= 108)) {
            lookupValue = 1270;
        } else if ((distanceFromBlueGoal > 108) && (distanceFromBlueGoal <= 120)) {
            lookupValue = 1300;
        } else if ((distanceFromBlueGoal > 120) && (distanceFromBlueGoal <= 132)) {
            lookupValue = 1340;
        } else if ((distanceFromBlueGoal > 132) && (distanceFromBlueGoal <= 144)) {
            lookupValue = 1380;
        } else if ((distanceFromBlueGoal > 144) && (distanceFromBlueGoal <= 156)) {
            lookupValue = 1420;
        } else if ((distanceFromBlueGoal > 156) && (distanceFromBlueGoal <= 168)) {
            lookupValue = 1460;
        } else if ((distanceFromBlueGoal > 168) && (distanceFromBlueGoal <= 180)) {
            lookupValue = 1500;
        } else if (distanceFromBlueGoal > 180) {
            lookupValue = 1540;
        }

        slightOffsetBlue = new Pose(
                follower.getPose().getX() + 0.5,  // tiny offset to avoid zero length
                follower.getPose().getY() + 0.5,
                angleTowardsBlueGoal
        );
        slightOffsetRed = new Pose(
                follower.getPose().getX() + 0.5,  // tiny offset to avoid zero length
                follower.getPose().getY() + 0.5,
                angleTowardsRedGoal
        );

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
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    turn,
                    true // Robot Centric
            ); else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    turn, // don't slow down auto aimer
                    true // Robot Centric
            );

            // only localize when not in the middle of a path
            localizer.update();

        }

        if (gamepad1.left_stick_button && !previousGamepad1.left_stick_button) {
            blueClose = () -> follower.pathBuilder() //Lazy Curve Generation
                    .addPath(new BezierLine(follower::getPose, slightOffsetBlue))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, angleTowardsBlueGoal, 0.0))
                    .build();
            follower.followPath(blueClose.get());
            automatedDrive = true;
        } else if (gamepad1.right_stick_button && !previousGamepad1.right_stick_button) {
            redClose = () -> follower.pathBuilder()
                    .addPath((new BezierLine(follower::getPose, slightOffsetRed)))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, angleTowardsRedGoal, 0.0))
                    .build();
            follower.followPath(redClose.get());
            automatedDrive = true;
        }

        //Stop automated following if the follower is done
        if (automatedDrive && ((gamepad1.dpad_down && !previousGamepad1.dpad_down)/*|| !follower.isBusy()*/)) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }
        if (gamepad1.a && !previousGamepad1.a) {
            //kV = 0.0002; //for tuning purposes
            kP = 0.007;
            launcher = 1120; //close launch zone velocity, was 1160
            launcherOn = !launcherOn;
            IntakeFSM.setGatekeepState(intakeFSM.GatekeepState.OFF);
        } else if (gamepad1.b && !previousGamepad1.b) {
            //kV = CONFIGkV; //for tuning purposes
            kP = CONFIGkP;
            //launcher = 1460; //far launch zone velocity
            launcher = lookupValue;
            launcherOn = !launcherOn;
            IntakeFSM.setGatekeepState(intakeFSM.GatekeepState.OFF);
        }
        /*if (gamepad1.dpad_right && !previousGamepad1.dpad_right) {
            launcher += 20;
        } else if (gamepad1.dpad_left && !previousGamepad1.dpad_left) {
            launcher -= 20;
        }*/
        if (gamepad1.dpad_right && !previousGamepad1.dpad_right) {
            follower.setPose(relocalizeBluePose);
        } else if (gamepad1.dpad_left && !previousGamepad1.dpad_left) {
            follower.setPose(relocalizeRedPose);
        }

        if ((motifTimer.getElapsedTimeSeconds() > 90) && !endgame) {
            gamepad1.rumble(500);
            endgame = true;
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
            IntakeFSM.setGatekeepState(intakeFSM.GatekeepState.ON);
        }

        /*TelemetryPacket packet = new TelemetryPacket();
        packet.put("Launcher Velocity", R.shooter.getVelocity());
        packet.put("Target Velocity", targetVelocity);
        dashboard.sendTelemetryPacket(packet);*/ // launcher tuning

        telemetry.addLine();

        telemetry.addData("targetVelocity", launcher);
        telemetry.addData("distanceFromBlueGoal", distanceFromBlueGoal);
        /*telemetry.addData("launchPower", R.shooter.getPower());
        telemetry.addData("launchVelo", R.shooter.getVelocity());
        telemetry.addData("transferVelocity", R.intakeHigh.getVelocity());
        telemetry.addData("automatedDrive", automatedDrive);*/
        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
        previousGamepad1.copy(currentGamepad1);
    }
}