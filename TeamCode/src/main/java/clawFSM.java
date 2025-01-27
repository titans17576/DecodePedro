import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import util.robot;

public class clawFSM {
    public enum ClawGrabState{
        CLOSED,
        OPEN,
    }
    public enum ClawWristState{
        DOWN,
        UP
    }


    // Position variables

    final double closed_position = 0;
    final double open_position = 0.33;
    final double down_position = 0.81; // Insert a number
    final double up_position = 1; // Insert a numbber

    // LiftState instance variable

    robot R;
    Telemetry telemetry;
    ClawGrabState clawGrabState;
    ClawWristState clawWristState;

    // Import opmode variables when instance is created
    public clawFSM(robot Robot, Telemetry t) {
        this(Robot, t, ClawGrabState.CLOSED, ClawWristState.DOWN);
    }
    public clawFSM(robot Robot, Telemetry t, ClawGrabState cG, ClawWristState cW) {
        R = Robot;
        telemetry = t;
        clawWristState = cW;
        clawGrabState = cG;
    }
    //public void initialize() {
    //}

    // Method to move to a targeted position
    private void moveGrabTo(Double position) {
        R.claw.setPosition(position);
    }
    private void moveWristTo(Double position) {
        R.specArm.setPosition(position);
    }

    // Method to add encoders and status to telemetry
    private void updateTelemetry(String status) {
        // Add lift position to telemetry
        telemetry.addData("Status of Claw", status);
    }

    // Update method for teleop implementation
    public void teleopUpdate(Gamepad currentGamepad, Gamepad previousGamepad) {
        telemetry.addLine("Lift Data");

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
    }
    public void update(){
        switch(clawGrabState) {
            case CLOSED:
                moveGrabTo(closed_position);
                break;
            case OPEN:
                moveGrabTo(open_position);
                break;
        }
        switch(clawWristState){
            case DOWN:
                moveWristTo(down_position);
                break;
            case UP:
                moveWristTo(up_position);
                break;
        }
    }
    public void setGrabState(ClawGrabState state){
        clawGrabState = state;
    }
    public void setWristState(ClawWristState state){
        clawWristState = state;
    }
}

