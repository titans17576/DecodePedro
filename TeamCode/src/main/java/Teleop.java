import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * This is an example teleop that showcases movement and robot-centric driving.
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
    private liftFSM LiftFSM;

    private final Pose startPose = new Pose(0,0,0);

    /** This method is call once when init is played, it initializes the follower **/
    @Override
    public void init() {
        Constants.setConstants(FConstants.class,LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();
        currentGamepad2 = new Gamepad();
        previousGamepad2 = new Gamepad();

        R = new robot(hardwareMap);
        LiftFSM = new liftFSM(R, telemetry, currentGamepad1,previousGamepad1);
    }

    /** This method is called continuously after Init while waiting to be started. **/
    @Override
    public void init_loop() {
    }

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        follower.startTeleopDrive();
        LiftFSM.initialize();
    }

    /** This is the main loop of the opmode and runs continuously after play **/
    @Override
    public void loop() {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);
        /* Update Pedro to move the robot based on:
        - Forward/Backward Movement: -gamepad1.left_stick_y
        - Left/Right Movement: -gamepad1.left_stick_x
        - Turn Left/Right Movement: -gamepad1.right_stick_x
        - Robot-Centric Mode: true
        */

        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();

        LiftFSM.teleopUpdate();

        if (gamepad1.a && !previousGamepad1.a) {
            R.claw.setPosition(0.2);
        } else if (gamepad1.b && !previousGamepad1.b) {
            R.claw.setPosition(0.41);
        }

        if (gamepad1.dpad_left && !previousGamepad1.dpad_left) {
            R.arm.setPosition(0.37);
        } else if (gamepad1.dpad_up && !previousGamepad1.dpad_up) {
            R.arm.setPosition(0.934);
        } else if (gamepad1.dpad_down && !previousGamepad1.dpad_down) {
            R.arm.setPosition(0.3);
        }

        if (gamepad2.left_bumper && !previousGamepad2.left_bumper) {
            R.extendo.setPosition(0.63);
        } else if (gamepad2.right_bumper && !previousGamepad2.right_bumper) {
            R.extendo.setPosition(0.5);
        }

        if (gamepad2.x && !previousGamepad2.x) {
            R.intakeWrist.setPosition(0.0);
        } else if (gamepad2.y && !previousGamepad2.y) {
            R.intakeWrist.setPosition(0.9);
        } else if (gamepad2.a && !previousGamepad2.a) {
            R.intakeWrist.setPosition(0.5);
        }

        if (gamepad2.dpad_left && !previousGamepad2.dpad_left) {
            R.intakeClaw.setPosition(0.8);
        } else if (gamepad2.dpad_up && !previousGamepad2.dpad_up) {
            R.intakeClaw.setPosition(0.5);
        } else if (gamepad2.dpad_down && !previousGamepad2.dpad_down) {
            R.intakeClaw.setPosition(1);
        }

        /* Telemetry Outputs of our Follower */
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

        /* Update Telemetry to the Driver Hub */
        telemetry.update();

    }

    /** We do not use this because everything automatically should disable **/
    @Override
    public void stop() {
    }
}