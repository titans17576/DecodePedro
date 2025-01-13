import static java.lang.Math.abs;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import util.robot;

public class clawFSM {
    public enum ClawState{
        CLOSED,
        OPEN,
    }


    // Position variables

    final double closed_position = 0.42;
    final double open_position = 0.2;

    // LiftState instance variable
    ClawState clawState = ClawState.CLOSED;

    robot R;
    Telemetry telemetry;
    Gamepad gamepad1;
    Gamepad previousGamepad1;

    // Import opmode variables when instance is created
    public clawFSM(robot Robot, Telemetry t, Gamepad g1, Gamepad gp1) {
        R = Robot;
        telemetry = t;
        gamepad1 = g1;
        previousGamepad1 = gp1;
    }
    //public void initialize() {
    //}

    // Method to move to a targeted position
    private void moveTo(Double position) {
        R.claw.setPosition(position);
    }

    // Method to add encoders and status to telemetry
    private void updateTelemetry(String status) {
        // Add lift position to telemetry
        telemetry.addData("Status of Claw", status);
    }

    // Update method for teleop implementation
    public void teleopUpdate() {
        telemetry.addLine("Lift Data");

        switch (clawState) {
            // Lift set to 0
            case CLOSED:
                telemetry.addData("Claw Moved", "TRUE");
            // State inputs
                if (gamepad1.a && !previousGamepad1.a) {
                    setState(ClawState.OPEN);
                }
                updateTelemetry("CLOSED");
                break;
            case OPEN:
                telemetry.addData("Claw Moved", "TRUE");
                if (gamepad1.a && !previousGamepad1.a) {
                    setState(ClawState.CLOSED);
                }
                updateTelemetry("OPEN");
                break;
        }
        update();
    }
    public void testUpdate() {
        updateTelemetry("Test");
        if (gamepad1.right_bumper && !previousGamepad1.right_bumper) {
            R.liftMotor.setTargetPosition(6000);
            R.liftMotor.setPower(1);
            R.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        if (gamepad1.left_bumper && !previousGamepad1.left_bumper) {
            R.liftMotor.setTargetPosition(0);
            R.liftMotor.setPower(0.8);
            R.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        if (R.liftMotor.getCurrentPosition() < 30 && R.liftMotor.getTargetPosition() != 6000){
            R.liftMotor.setPower(0);
            R.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // hi

        }

    }
    public void update(){
        switch(clawState) {
            case CLOSED:
                moveTo(closed_position);
                break;
            case OPEN:
                moveTo(open_position);
                break;

        }
    }
    public void setState(ClawState state){
        clawState = state;
    }
}

