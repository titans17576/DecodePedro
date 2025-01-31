import static java.lang.Math.abs;




import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import util.robot;

public class intakeFSM {
    public enum ExtenderState{
        EXTEND,
        RETRACT
    }
    public enum ClawState{
        OPEN,
        CLOSE
    }

    public enum RollerState{
        ZERO,
        MID,
        DROP
    }
    // Position variables

    final double extend_position = 0.3;
    final double retract_position = 0.9;
    final double open_position = 0.9;
    final double close_position = 0.9;
    final double zero_position = 0.9;
    final double mid_position = 0.9;
    final double drop_position = 0.9;

    // LiftState instance variable

    robot R;
    Telemetry telemetry;
    ExtenderState extenderState;
    ClawState clawState;
    RollerState rollerState;


    // Import opmode variables when instance is created
    public intakeFSM(robot Robot, Telemetry t) {
        this(Robot, t, ExtenderState.RETRACT, ClawState.CLOSE,  RollerState.ZERO);
    }
    public intakeFSM(robot Robot, Telemetry t, ExtenderState eS, ClawState cS, RollerState rS) {
        R = Robot;
        telemetry = t;
        extenderState = eS;
        clawState = cS;
        rollerState = rS;

    }
    //public void initialize() {
    //}

    // Method to move to a targeted position
    private void moveExtenderTo(Double position) {
        R.extendo.setPosition(position);
    }
    private void moveClawTo(Double position) {
        R.intakeClaw.setPosition(position);
    }
    private void moveRollerTo(Double position) {
        R.intakeWrist.setPosition(position);
    }

    // Method to add encoders and status to telemetry
    private void updateTelemetry(String status) {
        // Add lift position to telemetry
        telemetry.addData("Status of Arm", status);
    }

    // Update method for teleop implementation
    public void teleopUpdate(Gamepad currentGamepad, Gamepad previousGamepad) {
        telemetry.addLine("Lift Data");

        switch (extenderState) {
            // Lift set to 0
            case EXTEND:
                telemetry.addData("Scoop Moved", "TRUE");
                // State inputs
                if (currentGamepad.left_bumper && !previousGamepad.left_bumper) {
                    setExtenderState(ExtenderState.RETRACT);
                }
                updateTelemetry("SCORED");
                break;
            case RETRACT:
                telemetry.addData("Scoop Moved", "TRUE");
                if (currentGamepad.right_bumper && !previousGamepad.right_bumper) {
                    setExtenderState(ExtenderState.EXTEND);
                }
                updateTelemetry("WAITING");
                break;
        }
        switch (clawState) {
            // Lift set to 0
            case OPEN:
                telemetry.addData("Scoop Moved", "TRUE");
                // State inputs
                if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {
                    setClawState(ClawState.CLOSE);
                }
                updateTelemetry("SCORED");
                break;
            case CLOSE:
                telemetry.addData("Scoop Moved", "TRUE");
                if (currentGamepad.dpad_down && !previousGamepad.dpad_down) {
                    setClawState(ClawState.OPEN);
                }
                updateTelemetry("WAITING");
                break;
        }
        switch (rollerState) {
            // Lift set to 0
            case ZERO:
                telemetry.addData("Scoop Moved", "TRUE");
                // State inputs
                if (currentGamepad.a && !previousGamepad.a) {
                    setRollerState(RollerState.MID);
                }
                updateTelemetry("SCORED");
                break;
            case MID:
                telemetry.addData("Scoop Moved", "TRUE");
                if (currentGamepad.x && !previousGamepad.x) {
                    setRollerState(RollerState.ZERO);
                }else if(currentGamepad.y && !previousGamepad.y){
                setRollerState(RollerState.DROP);
            }
                updateTelemetry("WAITING");
                break;
            case DROP:
                if (currentGamepad.y && !previousGamepad.y) {
                    setRollerState(RollerState.MID);
                }
                updateTelemetry("SCORED");
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
        switch(extenderState) {
            case EXTEND:
                moveExtenderTo(extend_position);
                break;
            case RETRACT:
                moveExtenderTo(retract_position);
                break;

        }
        switch(clawState) {
            case OPEN:
                moveClawTo(open_position);
                break;
            case CLOSE:
                moveClawTo(close_position);
                break;

        }
        switch(rollerState) {
            case ZERO:
                moveRollerTo(zero_position);
                break;
            case MID:
                moveRollerTo(mid_position);
                break;
            case DROP:
                moveRollerTo(drop_position);
                break;

        }
    }
    public void setExtenderState(ExtenderState state){
        extenderState = state;
    }
    public void setClawState(ClawState state){
        clawState = state;
    }
    public void setRollerState(RollerState state){
        rollerState = state;
    }
}

