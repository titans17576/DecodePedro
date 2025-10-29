import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import pedroPathing.constants.Constants;
import util.robot;

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


    private final Pose startPose = new Pose(0, 0, 0);
    private double defaultSpeed = 1;
    private double highSpeed = 1;

    /**
     * This method is call once when init is played, it initializes the follower
     **/
    @Override
    public void init() {
        /*follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);*/
        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();
        currentGamepad2 = new Gamepad();
        previousGamepad2 = new Gamepad();

        R = new robot(hardwareMap);
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
        /*follower.startTeleopDrive();*/

        /*R.intakeArm.setPosition(0.55);
        R.extendo.setPosition(0.14);
        R.intakeWrist1.setPosition(0.1);*/
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
        double speed = (gamepad1.x) ? highSpeed : defaultSpeed;
        //follower.setTeleOpMovementVectors(-gamepad1.left_stick_y*speed, -gamepad1.left_stick_x*speed, -gamepad1.right_stick_x*speed, true);
        /*follower.update();*/

        // ticks_per_second = power * (max_rpm / 60) * ticks_per_revolution

        if (gamepad1.b && !previousGamepad1.b) {
            // R.shooter.setPower(0.64);
            R.shooter.setVelocity(powerToTicksPerSecond(0.64));
        }
        else if (gamepad1.a && !previousGamepad1.a){
            // R.shooter.setPower(0.6);
            R.shooter.setVelocity(powerToTicksPerSecond(0.6));
        }

        if (gamepad1.x && !previousGamepad1.x) {
            // R.intake.setPower(1);
            R.shooter.setVelocity(powerToTicksPerSecond(1));
        }
        else if (gamepad1.y && !previousGamepad1.y){
            // R.intake.setPower(0);
            R.shooter.setVelocity(powerToTicksPerSecond(0));
        }


        /* Telemetry Outputs of our Follower
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Lift Ticks", R.liftMotor.getCurrentPosition());
        Update Telemetry to the Driver Hub
        telemetry.update();*/
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);
    }

    private double powerToTicksPerSecond(double power) {
        return power * ((double) 6000 / 60) * 28;
    }
}