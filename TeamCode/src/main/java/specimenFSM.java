import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import util.robot;

public class specimenFSM {

    public enum ClawGrabState{
        CLOSED,
        OPEN,
    }
    public enum ClawWristState{
        DOWN,
        MID,

        UP
    }
    public enum LiftState {
        ZERO,
        LOW,
        MID,
        HIGH
    }


    // Position variables

    final double claw_closed_position = 0;
    final double claw_open_position = 0.33;
    final double claw_down_position = 1; // Insert a number
    final double claw_mid_position = 0.94;
    final double claw_up_position = 0.81; // Insert a number

    final int position_tolerance = 15;
    final int lift_zero_position = 0;
    int lift_low_position = 300;
    final int lift_mid_position = 850; // max we could reach was like 1500 ticks so idk
    final int lift_high_position = 1450;
    public boolean actionBusy = false;



    // LiftState instance variable

    robot R;
    Telemetry telemetry;
    ClawGrabState clawGrabState;
    ClawWristState clawWristState;
    LiftState liftState = LiftState.ZERO;


    // Import opmode variables when instance is created
    public specimenFSM(robot Robot, Telemetry t) {
        this(Robot, t, ClawGrabState.CLOSED, ClawWristState.DOWN, LiftState.ZERO);
    }
    public specimenFSM(robot Robot, Telemetry t, ClawGrabState cG, ClawWristState cW, LiftState lS) {
        R = Robot;
        telemetry = t;
        clawWristState = cW;
        clawGrabState = cG;
        liftState = lS;
    }
    public void initialize() {
        R.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Method to move to a targeted position
    private void moveGrabTo(Double position) {
        R.claw.setPosition(position);
    }
    private void moveWristTo(Double position) {
        R.specArm.setPosition(position);
    }
    private void moveLiftTo(int position, double power) {
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
    private void updateTelemetry(String status) {
        // Add lift position to telemetry
        telemetry.addData("Status of Claw", status);
    }

    // Update method for teleop implementation
    public void teleopUpdate(Gamepad currentGamepad, Gamepad previousGamepad) {
        telemetry.addLine("Lift Data");
        if (currentGamepad.right_bumper && !previousGamepad.right_bumper) {
            setLiftState(LiftState.MID);
            setWristState(ClawWristState.UP);
        }
        if (currentGamepad.left_trigger >= 0.5 && previousGamepad.left_trigger < 0.5) {
            setLiftState(LiftState.HIGH);
            setWristState(ClawWristState.DOWN);
        }
        if (currentGamepad.right_trigger >= 0.5 && previousGamepad.right_trigger < 0.5) {
            setLiftState(LiftState.LOW);
        }
        if (currentGamepad.left_bumper && !previousGamepad.left_bumper) {
            setWristState(ClawWristState.DOWN);
            setLiftState(LiftState.ZERO);
        }

        switch (clawGrabState) {
            // Lift set to 0
            case CLOSED:
                telemetry.addData("Claw Moved", "TRUE");
                // State inputs
                if (currentGamepad.a && !previousGamepad.a) {
                    setGrabState(ClawGrabState.OPEN);
                }
                updateTelemetry("CLOSED");
                break;
            case OPEN:
                telemetry.addData("Claw Moved", "TRUE");
                if (currentGamepad.a && !previousGamepad.a) {
                    setGrabState(ClawGrabState.CLOSED);
                }
                updateTelemetry("OPEN");
                break;
        }

        switch(clawWristState){
            case DOWN:
                if (currentGamepad.dpad_left && !previousGamepad.dpad_left){
                    setWristState(ClawWristState.UP);
                }
                break;
            case UP:
                if (currentGamepad.dpad_right && !previousGamepad.dpad_right){
                    setWristState(ClawWristState.DOWN);
                }
                break;

        }

        update();
    }
    public void testUpdate(Gamepad currentGamepad, Gamepad previousGamepad) {
        updateTelemetry("Test");
        if (currentGamepad.a && !previousGamepad.a) {
            R.claw.setPosition(0.2);
        } else if (currentGamepad.b && !previousGamepad.b) {
            R.claw.setPosition(0.42);
        }
        if (currentGamepad.dpad_right && !previousGamepad.dpad_right) {
            R.specArm.setPosition(0.81);
        } else if (currentGamepad.dpad_left && !previousGamepad.dpad_left) {
            R.specArm.setPosition(1);
        }
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
    public void update(){
        switch(clawGrabState) {
            case CLOSED:
                moveGrabTo(claw_closed_position);
                break;
            case OPEN:
                moveGrabTo(claw_open_position);
                break;
        }
        switch(clawWristState){
            case DOWN:
                moveWristTo(claw_down_position);
                break;
            case UP:
                moveWristTo(claw_up_position);
                break;
            case MID:
                moveWristTo(claw_mid_position);
                break;
        }

        switch (liftState){
            case ZERO:
                moveLiftTo(lift_zero_position,0.8);
                break;
            case LOW:
                moveLiftTo(lift_low_position,1);
                break;
            case MID:
                moveLiftTo(lift_mid_position,1);
                break;
            case HIGH:
                moveLiftTo(lift_high_position,1);
                break;
        }

    }
    public void setGrabState(ClawGrabState state){
        clawGrabState = state;
    }
    public void setWristState(ClawWristState state){
        clawWristState = state;
    }
    public void setLiftState(LiftState state){
        liftState = state;
    }

    public boolean actionNotBusy(){
        return !actionBusy;
    }
}
