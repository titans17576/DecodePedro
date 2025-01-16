import static java.lang.Math.abs;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import util.robot;

public class scoopFSM {
    public enum ScoopState{
        CLOSED,
        OPEN,
    }


    // Position variables

    final double closed_position = 0.42;
    final double open_position = 0.2;

    // LiftState instance variable

    robot R;
    Telemetry telemetry;
    ScoopState scoopState;

    // Import opmode variables when instance is created
    public scoopFSM(robot Robot, Telemetry t) {
        this(Robot, t, ScoopState.CLOSED);
    }
    public scoopFSM(robot Robot, Telemetry t, ScoopState cS) {
        R = Robot;
        telemetry = t;
        scoopState = cS;
    }
    //public void initialize() {
    //}

    // Method to move to a targeted position
    private void moveTo(Double position) {
        R.arm.setPosition(position);
    }

    // Method to add encoders and status to telemetry
    private void updateTelemetry(String status) {
        // Add lift position to telemetry
        telemetry.addData("Status of Arm", status);
    }

    // Update method for teleop implementation
    public void teleopUpdate(Gamepad currentGamepad, Gamepad previousGamepad) {
        telemetry.addLine("Lift Data");

        switch (scoopState) {
            // Lift set to 0
            case CLOSED:
                telemetry.addData("Scoop Moved", "TRUE");
                // State inputs
                if (currentGamepad.a && !previousGamepad.a) {
                    setState(scoopState.OPEN);
                }
                updateTelemetry("CLOSED");
                break;
            case OPEN:
                telemetry.addData("Scoop Moved", "TRUE");
                if (currentGamepad.a && !previousGamepad.a) {
                    setState(ScoopState.CLOSED);
                }
                updateTelemetry("OPEN");
                break;
        }
        update();
    }
    public void testUpdate(Gamepad currentGamepad, Gamepad previousGamepad) {
        updateTelemetry("Test");
        if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {
            R.arm.setPosition(0.3);
        } else if (currentGamepad.dpad_down && !previousGamepad.dpad_down) {
            R.arm.setPosition(0.95);
        }
    }
    public void update(){
        switch(scoopState) {
            case CLOSED:
                moveTo(closed_position);
                break;
            case OPEN:
                moveTo(open_position);
                break;

        }
    }
    public void setState(ScoopState state){
        scoopState = state;
    }
}

