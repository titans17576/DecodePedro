import static java.lang.Math.abs;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import util.robot;

public class clawFSM {
    public enum ClawState{
        ZERO,
        MID,
        OPEN,
    }


    // Position variables

    final double zero_position = 0;
    final double mid_position = 0.47;
    final double open_position = 0.55;

    // LiftState instance variable
    ClawState clawState = ClawState.ZERO;

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
        // Add encoder position to telemetry
        telemetry.addData("Lift Ticks", R.liftMotor.getCurrentPosition());
        // Add lift position to telemetry
        telemetry.addData("Status of Lift", status);
    }

    // Update method for teleop implementation
    public void teleopUpdate() {
        telemetry.addLine("Lift Data");

        switch (clawState) {
            // Lift set to 0
            case ZERO:

                moveTo(zero_position);

                telemetry.addData("Claw Moved", "TRUE");


                // State inputs
                if (gamepad1.dpad_up && !previousGamepad1.dpad_up) {
                    clawState = ClawState.MID;
                    telemetry.addData("Move Requested", "TRUE");
                } else {
                    telemetry.addData("Move Requested", "FALSE");
                }

                updateTelemetry("Zero");

                break;


            // Lift set to 3/3
            case MID:

                moveTo(mid_position);
                telemetry.addData("Claw Moved", "TRUE");


                // State inputs
                if (gamepad1.dpad_down && !previousGamepad1.dpad_down) {
                    clawState = ClawState.ZERO;
                    telemetry.addData("Move Requested", "TRUE");
                } else {
                    telemetry.addData("Move Requested", "FALSE");
                }

                if (gamepad1.dpad_up && !previousGamepad1.dpad_up) {
                    moveTo(open_position);
                    telemetry.addData("Lift Moved", "TRUE");
                } else {
                    telemetry.addData("Move Requested", "FALSE");
                }


                updateTelemetry("HIGH");

                break;
            case OPEN:
                moveTo(open_position);
                telemetry.addData("Claw Moved", "TRUE");
                if (gamepad1.dpad_down && !previousGamepad1.dpad_down) {
                    clawState = ClawState.MID;
                    telemetry.addData("Move Requested", "TRUE");
                } else {
                    telemetry.addData("Move Requested", "FALSE");
                }
                break;


        }
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
        public void update(){
            switch(clawState) {
                case ZERO:
                    moveTo(zero_position);
                    break;
                case MID:
                    moveTo(mid_position);
                    break;
                case OPEN:
                    moveTo(open_position);
                    break;
            
            }
        }
    }
}

