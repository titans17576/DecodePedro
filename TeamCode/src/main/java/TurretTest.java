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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static pedroPathing.ConfigFile.CONFIGkP;
import static pedroPathing.ConfigFile.CONFIGkV;
import static pedroPathing.ConfigFile.CONFIGkS;
import static pedroPathing.ConfigFile.LOOPTIME;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import pedroPathing.constants.Constants;
import util.robot;
import java.util.function.Supplier;

/**
 * This is an example teleop that showcases movement and util.robot-centric driving.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 12/30/2024
 */

@TeleOp(name = "TurretTest")
public class TurretTest extends OpMode {
    private Gamepad currentGamepad1;
    private Gamepad previousGamepad1;
    private Gamepad currentGamepad2;
    private Gamepad previousGamepad2;

    public Servo hoodServo;
    // Top/Bottom refers to motor position
    public DcMotorEx shooterTop;
    public DcMotorEx shooterBottom;

    private boolean shooterActive = false;

    /**
     * This method is call once when init is played, it initializes the follower
     **/
    @Override
    public void init() {
        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();
        currentGamepad2 = new Gamepad();
        previousGamepad2 = new Gamepad();

        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        shooterTop = hardwareMap.get(DcMotorEx.class, "shooterTop");
        shooterBottom = hardwareMap.get(DcMotorEx.class, "shooterBottom");
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
    }

    /**
     * Called once at the end of the OpMode
     */
    @Override
    public void stop() {
    }

    /**
     * This is the main loop of the opmode and runs continuously after play
     **/
    @Override
    public void loop() {
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) hoodServo.setPosition(1);
        if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) hoodServo.setPosition(0.46);

        if (currentGamepad1.a && !previousGamepad1.a) shooterActive = !shooterActive;
        if (shooterActive) {
            final double speed = 0.7;
            shooterTop.setPower(speed);
            shooterBottom.setPower(-speed);
        } else {
            shooterTop.setPower(0);
            shooterBottom.setPower(0);
        }

        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);
    }
}