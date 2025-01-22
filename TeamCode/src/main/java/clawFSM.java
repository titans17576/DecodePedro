import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import util.robot;

public class clawFSM {
    public enum ClawState{
        CLOSED,
        OPEN,
    }


    // Position variables

    final double closed_position = 0;
    final double open_position = 0.35;

    // LiftState instance variable

    robot R;
    Telemetry telemetry;
    ClawState clawState;

    // Import opmode variables when instance is created
    public clawFSM(robot Robot, Telemetry t) {
        this(Robot, t, ClawState.CLOSED);
    }
    public clawFSM(robot Robot, Telemetry t, ClawState cS) {
        R = Robot;
        telemetry = t;
        clawState = cS;
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
    public void teleopUpdate(Gamepad currentGamepad, Gamepad previousGamepad) {
        telemetry.addLine("Lift Data");

        switch (clawState) {
            // Lift set to 0
            case CLOSED:
                telemetry.addData("Claw Moved", "TRUE");
            // State inputs
                if (currentGamepad.a && !previousGamepad.a) {
                    setState(ClawState.OPEN);
                }
                updateTelemetry("CLOSED");
                break;
            case OPEN:
                telemetry.addData("Claw Moved", "TRUE");
                if (currentGamepad.a && !previousGamepad.a) {
                    setState(ClawState.CLOSED);
                }
                updateTelemetry("OPEN");
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

