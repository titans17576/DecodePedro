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
        HIGH,
        BAD
    }
    public enum SpecArmState{
        GRAB,
        HANG,
        TRANSFER,
        DEPOSIT
    }


    // Position variables

    final double claw_closed_position = 0.52;
    final double claw_open_position = 0.18;
    final double claw_down_position = 0.94; // Insert a number
    final double claw_mid_position = 0.94;
    final double claw_up_position = 0.70; // Insert a number
    final double specArm_grab_position = 0.12;
    final double specArm_hang_position = 0.76;
    final double specArm_transfer_position = 0.78;
    final double specArm_deposit_position = 0.3;

    final int position_tolerance = 15;
    final int lift_zero_position = 0;
    final int lift_low_position = 300;
    final int lift_mid_position = 1125; // max we could reach was like 1500 ticks so idk
    final int lift_high_position = 1950;
    final int lift_bad_position = -2000;
    public boolean actionBusy = false;



    // LiftState instance variable

    robot R;
    Telemetry telemetry;
    ClawGrabState clawGrabState;
    ClawWristState clawWristState;
    SpecArmState specArmState;
    LiftState liftState = LiftState.ZERO;


    // Import opmode variables when instance is created
    public specimenFSM(robot Robot, Telemetry t) {
        this(Robot, t, ClawGrabState.CLOSED, ClawWristState.DOWN, SpecArmState.HANG, LiftState.ZERO);
    }
    public specimenFSM(robot Robot, Telemetry t, ClawGrabState cG, ClawWristState cW, SpecArmState sA, LiftState lS) {
        R = Robot;
        telemetry = t;
        clawWristState = cW;
        clawGrabState = cG;
        specArmState = sA;
        liftState = lS;
    }
    public void initialize() {
        R.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        R.liftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        R.specArm.setPosition(claw_down_position);
    }

    // Method to move to a targeted position
    private void moveGrabTo(Double position) {
        R.claw.setPosition(position);
    }
    private void moveWristTo(Double position) {
        R.specArm.setPosition(position);
    }
    private void moveArmTo(Double position) {
        R.specArm2.setPosition(position);
    }
    private void moveLiftTo(int position, double power) {
        if (abs(R.liftMotor.getCurrentPosition() - position) > position_tolerance) {
            R.liftMotor.setTargetPosition(position);
            R.liftMotor.setPower(power);
            R.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            R.liftMotor2.setTargetPosition(position);
            R.liftMotor2.setPower(power);
            R.liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
        /*updateTelemetry("Test");*/
        if (currentGamepad.right_bumper && !previousGamepad.right_bumper) {
            setLiftState(LiftState.MID);
            moveArmTo(specArm_hang_position);
            moveWristTo(claw_up_position);
        }
        if (currentGamepad.left_trigger >= 0.5 && previousGamepad.left_trigger < 0.5) {
            setLiftState(LiftState.HIGH);
            moveWristTo(claw_down_position);
        }
        if (currentGamepad.right_trigger >= 0.5 && previousGamepad.right_trigger < 0.5) {
            setLiftState(LiftState.LOW);
        }
        if (currentGamepad.left_bumper && !previousGamepad.left_bumper) {
            moveWristTo(claw_down_position);
            moveArmTo(specArm_grab_position);
            setLiftState(LiftState.ZERO);
        }
        if (currentGamepad.dpad_right && !previousGamepad.dpad_right) {
            setLiftState(LiftState.BAD);
        }
        if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {
            moveArmTo(specArm_transfer_position);
        } else if (currentGamepad.dpad_down && !previousGamepad.dpad_down) {
            moveArmTo(specArm_deposit_position);
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
        if (currentGamepad.x && !previousGamepad.x){
            moveWristTo(R.specArm.getPosition() + 0.04);
        }
        if (currentGamepad.y && !previousGamepad.y){
            moveWristTo(R.specArm.getPosition() - 0.04);
        }
        testUpdate();
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
        switch(specArmState){
            case GRAB:
                moveArmTo(specArm_grab_position);
                break;
            case HANG:
                moveArmTo(specArm_hang_position);
                break;
            case TRANSFER:
                moveArmTo(specArm_transfer_position);
                break;
            case DEPOSIT:
                moveArmTo(specArm_deposit_position);
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
            case BAD:
                moveLiftTo(lift_bad_position,0.8);
                break;
        }

    }
    public void testUpdate() {
        switch(clawGrabState) {
            case CLOSED:
                moveGrabTo(claw_closed_position);
                break;
            case OPEN:
                moveGrabTo(claw_open_position);
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
            case BAD:
                moveLiftTo(lift_bad_position,0.8);
                break;
        }
    }


    public void setGrabState(ClawGrabState state){
    clawGrabState = state;
    }
    public void setWristState(ClawWristState state){
        clawWristState = state;
    }
    public void setSpecArmState(SpecArmState state){
        specArmState = state;
    }
    public void setLiftState(LiftState state){
        liftState = state;
    }

    public boolean actionNotBusy(){
        return !actionBusy;
    }
}
