import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import util.robot;

public class liftFSM {
    // Enum for state memory
    public enum LiftState {
        ZERO,
        LOW,
        MID,
        HIGH
    }

    // Position variables
    final int position_tolerance = 15;
    final int zero_position = 0;
    int low_position = 900;
    final int mid_position = 1400; // max we could reach was like 1500 ticks so idk
    final int high_position = 2600;

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
        if (abs(R.liftMotor.getCurrentPosition() - position) > position_tolerance) {
            R.liftMotor.setTargetPosition(position);
            R.liftMotor.setPower(0.8);
            R.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //R.liftMotor.setPower(0);
        }
    }
    private void moveTo(int position, double power) {
        if (abs(R.liftMotor.getCurrentPosition() - position) > position_tolerance) {
            R.liftMotor.setTargetPosition(position);
            R.liftMotor.setPower(power);
            R.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //R.liftMotor.setPower(0);
        }
    }

    // Method to add encoders and status to telemetry
    public void updateTelemetry(String status) {
        // Add encoder position to telemetry
        telemetry.addData("Lift Ticks", R.liftMotor.getCurrentPosition());
        // Add lift position to telemetry
        telemetry.addData("Status of Lift", status);
    }

    public void update(){
        switch (liftState){
            case ZERO:
                moveTo(zero_position,1);
                break;
            case LOW:
                moveTo(low_position,1);
                break;
            case MID:
                moveTo(mid_position,1);
                break;
            case HIGH:
                moveTo(high_position,0.8);
                break;
        }
    }
    // Update method for teleop implementation
    public void teleopUpdate() {
        telemetry.addLine("Lift Data");

        if (gamepad1.right_bumper && !previousGamepad1.right_bumper) {
            setState(LiftState.MID);
        }
        if (gamepad1.left_trigger >= 0.5 && previousGamepad1.left_trigger < 0.5) {
            setState(LiftState.HIGH);
        }
        if (gamepad1.right_trigger >= 0.5 && previousGamepad1.right_trigger < 0.5) {
           setState(LiftState.LOW);
        }
        if (gamepad1.left_bumper && !previousGamepad1.left_bumper) {
            setState(LiftState.ZERO);

        }
        update();
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

    public void setState(LiftState state){
        liftState = state;
    }
}