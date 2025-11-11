package pedroPathing;

import static pedroPathing.ConfigFile.CONFIGkP;
import static pedroPathing.ConfigFile.CONFIGkI;
import static pedroPathing.ConfigFile.CONFIGkD;
import static pedroPathing.ConfigFile.loopTime;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "pidTest", group = "NoahIsSilly")
public class pidTest extends OpMode {

    private DcMotorEx launcher, launcher2;
    private FtcDashboard dashboard;

    private double kP, kI, kD;
    double error;

    private double integralSum = 0;
    private double lastError = 0;
    private double pidOutput = 0; // current motor power

    private ElapsedTime pidTimer = new ElapsedTime();

    private double targetVelocity = 0;
    private boolean launcherOn = false;
    private boolean intakeOn = false;
    private boolean lastX = false;
    private boolean lastA = false;

    private final double LOOP_TIME = loopTime;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();

        launcher = hardwareMap.get(DcMotorEx.class, "shooter");
        launcher.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcher.setDirection(DcMotorSimple.Direction.FORWARD);

        pidTimer.reset();
    }


    private double runPID(double target, double current, double currentPower) {
        error = target - current;
        integralSum += error * LOOP_TIME;
        double derivative = (error - lastError) / LOOP_TIME;
        double deltaPower = (kP * error) + (kI * integralSum) + (kD * derivative);

        lastError = error;

        return currentPower + deltaPower; // relative adjustment
    }



    @Override
    public void loop() {
        double currentVelocity = launcher.getVelocity();
        error = targetVelocity - currentVelocity;

            kP = CONFIGkP;
            kI = CONFIGkI;
            kD = CONFIGkD;


        if (gamepad1.x && !lastX) {
            launcherOn = !launcherOn;
        }



        if (launcherOn) {
            targetVelocity = 1300; // ticks/sec
            if (pidTimer.seconds() >= LOOP_TIME) {
                pidOutput = runPID(targetVelocity, currentVelocity, pidOutput);
                pidOutput = Math.max(0.0, Math.min(1.0, pidOutput)); // clamp to [0,1]
                launcher.setPower(pidOutput);
                pidTimer.reset();
            }
        } else {
            launcher.setPower(0);
            targetVelocity = 0;
            pidOutput = 0;
            integralSum = 0;
            lastError = 0;
        }


        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Launcher Velocity", currentVelocity);
        packet.put("Target Velocity", targetVelocity);
        dashboard.sendTelemetryPacket(packet);


        telemetry.addData("Launcher On", launcherOn);
        telemetry.addData("Launcher Target Velocity", targetVelocity);
        telemetry.addData("Launcher Velocity", currentVelocity);
        telemetry.addData("Motor Power", pidOutput);
        telemetry.addData("Error", targetVelocity - currentVelocity);
        telemetry.update();

        lastX = gamepad1.x;
        lastA = gamepad1.a;
    }

    @Override
    public void stop() {
        launcher.setPower(0);
    }
}