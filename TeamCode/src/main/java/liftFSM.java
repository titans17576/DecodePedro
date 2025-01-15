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
    int low_position = 1440;
    final int mid_position = 1600; // max we could reach was like 1500 ticks so idk
    final int high_position = 2600;

    // LiftState instance variable
    LiftState liftState = LiftState.ZERO;
    public boolean actionBusy = false;

    // OpMode variables
    robot R;
    Telemetry telemetry;

    // Import opmode variables when instance is created
    public liftFSM(robot Robot, Telemetry t) {
        this(Robot, t, LiftState.ZERO);
    }
    public liftFSM(robot Robot, Telemetry t, LiftState lS) {
        R = Robot;
        telemetry = t;
        liftState = lS;
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
            actionBusy = true;
        }
    }
    private void moveTo(int position, double power) {
        if (abs(R.liftMotor.getCurrentPosition() - position) > position_tolerance) {
            R.liftMotor.setTargetPosition(position);
            R.liftMotor.setPower(power);
            R.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //R.liftMotor.setPower(0);
            actionBusy = true;
        }
        else{
            actionBusy = false;
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
    public void teleopUpdate(Gamepad currentGamepad, Gamepad previousGamepad) {
        telemetry.addLine("Lift Data");

        if (currentGamepad.right_bumper && !previousGamepad.right_bumper) {
            setState(LiftState.MID);
        }
        if (currentGamepad.left_trigger >= 0.5 && previousGamepad.left_trigger < 0.5) {
            setState(LiftState.HIGH);
        }
        if (currentGamepad.right_trigger >= 0.5 && previousGamepad.right_trigger < 0.5) {
           setState(LiftState.LOW);
        }
        if (currentGamepad.left_bumper && !previousGamepad.left_bumper) {
            setState(LiftState.ZERO);

        }
        update();
    }
    public void testUpdate(Gamepad currentGamepad, Gamepad previousGamepad) {
        updateTelemetry("Test");
        if (currentGamepad.right_bumper && !previousGamepad.right_bumper) {
            R.liftMotor.setTargetPosition(1600);
            R.liftMotor.setPower(1);
            R.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            R.arm.setPosition(1);

        }
        if (currentGamepad.left_trigger >= 0.5 && previousGamepad.left_trigger < 0.5) {
            R.liftMotor.setTargetPosition(2600);
            R.liftMotor.setPower(1);
            R.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        if (currentGamepad.right_trigger >= 0.5 && previousGamepad.right_trigger < 0.5) {
            R.liftMotor.setTargetPosition(1440);
            R.liftMotor.setPower(1);
            R.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        if (currentGamepad.left_bumper && !previousGamepad.left_bumper) {
            R.liftMotor.setTargetPosition(0);
            R.liftMotor.setPower(0.8);
            R.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        if (R.liftMotor.getCurrentPosition() < 20 && R.liftMotor.getTargetPosition() == 0) {
            R.liftMotor.setPower(0);
            R.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        } else if (currentGamepad.y && !previousGamepad.y) {
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
    public boolean actionNotBusy() {
        return !actionBusy;
    }
}