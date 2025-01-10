import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import util.robot;

public class liftFSM {
    // Enum for state memory
    public enum LiftState {
        ZERO,
        HIGH
    }

    // Position variables
    final int position_tolerance = 15;
    final int zero_position = 0;
    // int low_position = 1500;
    //final int mid_position = 1442; // max we could reach was like 1500 ticks so idk
    final int high_position = 1980;

    // LiftState instance variable
    LiftState liftState = LiftState.ZERO;

    // OpMode variables
    robot R;
    Telemetry telemetry;
    Gamepad gamepad1;
    Gamepad previousGamepad1;

    // Import opmode variables when instance is created
    public liftFSM(robot Robot, Telemetry t, Gamepad g1, Gamepad gp1) {
        R = Robot;
        telemetry = t;
        gamepad1 = g1;
        previousGamepad1 = gp1;
    }
    public void initialize() {
        R.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Method to move to a targeted position
    private void moveTo(int position) {
        R.liftMotor.setTargetPosition(position);
        R.liftMotor.setPower(0.8);
        R.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        R.liftMotor.setPower(0);
    }

    // Method to add encoders and status to telemetry
    public void updateTelemetry(String status) {
        // Add encoder position to telemetry
        telemetry.addData("Lift Ticks", R.liftMotor.getCurrentPosition());
        // Add lift position to telemetry
        telemetry.addData("Status of Lift", status);
    }

    // Update method for teleop implementation
    public void teleopUpdate() {
        telemetry.addLine("Lift Data");

        switch (liftState) {
            // Lift set to 0
            case ZERO:
                // Check position and move if not at 0
                if (abs(R.liftMotor.getCurrentPosition() - zero_position) > position_tolerance) {

                    moveTo(zero_position);

                    telemetry.addData("Lift Moved", "TRUE");
                } else {
                    telemetry.addData("Lift Moved", "FALSE");
                }

                // State inputs
                if (gamepad1.dpad_up && !previousGamepad1.dpad_up) {
                    liftState = LiftState.HIGH;
                    telemetry.addData("Move Requested", "TRUE");
                } else {
                    telemetry.addData("Move Requested", "FALSE");
                }

                updateTelemetry("Zero");

                break;


            // Lift set to 3/3
            case HIGH:
                // Check position and move if not at high_position
                if (abs(R.liftMotor.getCurrentPosition() - high_position) > position_tolerance) {
                    moveTo(high_position);
                    telemetry.addData("Lift Moved", "TRUE");
                } else {
                    telemetry.addData("Lift Moved", "FALSE");
                }

                // State inputs
                if (gamepad1.dpad_down && !previousGamepad1.dpad_down) {
                    liftState = LiftState.ZERO;
                    telemetry.addData("Move Requested", "TRUE");
                } else {
                    telemetry.addData("Move Requested", "FALSE");
                }


                updateTelemetry("HIGH");

                break;

        }

    }
    public void testUpdate() {
        updateTelemetry("Test");
        if (gamepad1.right_bumper && !previousGamepad1.right_bumper) {
            R.liftMotor.setTargetPosition(1600);
            R.liftMotor.setPower(1);
            R.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            R.arm.setPosition(1);

        }
        if (gamepad1.left_trigger >= 0.5 && previousGamepad1.left_trigger < 0.5) {
            R.liftMotor.setTargetPosition(R.liftMotor.getTargetPosition() - 400);
            R.liftMotor.setPower(1);
            R.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        if (gamepad1.right_trigger >= 0.5 && previousGamepad1.right_trigger < 0.5) {
            R.liftMotor.setTargetPosition(R.liftMotor.getTargetPosition() + 200);
            R.liftMotor.setPower(1);
            R.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        if (gamepad1.left_bumper && !previousGamepad1.left_bumper) {
            R.liftMotor.setTargetPosition(0);
            R.liftMotor.setPower(0.8);
            R.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        if (R.liftMotor.getCurrentPosition() < 20 && R.liftMotor.getTargetPosition() == 0) {
            R.liftMotor.setPower(0);
            R.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        } else if (gamepad1.y && !previousGamepad1.y) {
            R.liftMotor.setPower(0);
            R.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if (R.liftMotor.getTargetPosition() < 0) {
            R.liftMotor.setTargetPosition(0);
        } else if (R.liftMotor.getTargetPosition() > 3000) {
            R.liftMotor.setTargetPosition(3000);
        }
    }
}