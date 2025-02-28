import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import util.robot;

public class intakeFSM {
    public enum ExtendoState{
        EXTEND,
        RETRACT
    }
    public enum ClawState{
        OPEN,
        CLOSE
    }
    public enum VerticalWristState{
        GRAB,
        TRANSFER
    }
    public enum HorizontalWristState{
        LEFT_RIGHT,
        UP_DOWN,
        DIAGONAL_RIGHT,
        DIAGONAL_LEFT
    }
    public enum ArmState{
        GRAB,
        HOVER,
        TRANSFER
    }
    // Position variables

    final double extend_position = 0.3;
    final double retract_position = 0.13;
    final double open_position = 0.5;
    final double close_position = 0.35;
    final double grab_position = 0.62;
    final double hover_position = 0.52;
    final double transfer_position = 0.31;
    final double verticalWrist_grab_position = 0.11;
    final double verticalWrist_transfer_position = 0.7;
    final double horizontalWrist_LR_position = 0.79;
    final double horizontalWrist_UD_position = 0.5;
    final double horizontalWrist_diagonalL_position = 0.65;
    final double horizontalWrist_diagonalR_position = 0.9;

    // LiftState instance variable

    robot R;
    Telemetry telemetry;
    ExtendoState extendoState;
    ClawState clawState;
    ArmState armState;
    VerticalWristState verticalWristState;
    HorizontalWristState horizontalWristState;


    // Import opmode variables when instance is created
    public intakeFSM(robot Robot, Telemetry t) {
        this(Robot, t, ExtendoState.RETRACT, ClawState.CLOSE,  ArmState.HOVER, VerticalWristState.GRAB, HorizontalWristState.LEFT_RIGHT);
    }
    public intakeFSM(robot Robot, Telemetry t, ExtendoState eS, ClawState cS, ArmState aS, VerticalWristState vW, HorizontalWristState hW) {
        R = Robot;
        telemetry = t;
        extendoState = eS;
        clawState = cS;
        armState = aS;
        verticalWristState = vW;
        horizontalWristState = hW;
    }
    public void initialize() {
        R.intakeArm.setPosition(0.55);
        R.extendo.setPosition(retract_position);
        R.intakeWrist1.setPosition(verticalWrist_grab_position);
    }

    // Method to move to a targeted position
    private void moveExtendoTo(Double position) {R.extendo.setPosition(position);}
    private void moveClawTo(Double position) {R.intakeClaw.setPosition(position);}
    private void moveArmTo(Double position) {R.intakeArm.setPosition(position);}
    private void moveVerticalWristTo(Double position) {R.intakeWrist1.setPosition(position);}
    private void moveHorizontalWristTo(Double position) {R.intakeWrist2.setPosition(position);}

    // Method to add encoders and status to telemetry
    private void updateTelemetry(String status) {
        // Add lift position to telemetry
        telemetry.addData("Status of Arm", status);
    }

    // Update method for teleop implementation
    public void teleopUpdate(Gamepad currentGamepad, Gamepad previousGamepad) {

        switch (extendoState) {
            // Lift set to 0
            case RETRACT:
                // State inputs
                if (currentGamepad.right_bumper && !previousGamepad.right_bumper) {
                    setExtendoState(ExtendoState.EXTEND);
                }
                break;
            case EXTEND:
                if (currentGamepad.left_bumper && !previousGamepad.left_bumper) {
                    setExtendoState(ExtendoState.RETRACT);
                }
                break;
        }
        switch (clawState) {
            case OPEN:
                // State inputs
                if (currentGamepad.b && !previousGamepad.b) {
                    setClawState(ClawState.CLOSE);
                }
                break;
            case CLOSE:
                if (currentGamepad.b && !previousGamepad.b) {
                    setClawState(ClawState.OPEN);
                }
                break;
        }
        if (currentGamepad.y && !previousGamepad.y) {
            setArmState(ArmState.TRANSFER);
            setVerticalWristState(VerticalWristState.TRANSFER);
            setHorizontalWristState(HorizontalWristState.LEFT_RIGHT);
        } else if (currentGamepad.a && !previousGamepad.a) {
            setArmState(ArmState.HOVER);
            setVerticalWristState(VerticalWristState.GRAB);
        } else if (currentGamepad.x && !previousGamepad.x) {
            setArmState(ArmState.GRAB);
            setVerticalWristState(VerticalWristState.GRAB);
        }

        switch (horizontalWristState) {
            case LEFT_RIGHT:
                if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {
                    setHorizontalWristState(HorizontalWristState.DIAGONAL_RIGHT);
                }
                break;
            case UP_DOWN:
                if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {
                    setHorizontalWristState(HorizontalWristState.DIAGONAL_LEFT);
                }
                break;
            case DIAGONAL_LEFT:
                if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {
                    setHorizontalWristState(HorizontalWristState.LEFT_RIGHT);
                }
                break;
            case DIAGONAL_RIGHT:
                if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {
                    setHorizontalWristState(HorizontalWristState.UP_DOWN);
                }
                break;
        }

        update();
    }
    public void testUpdate(Gamepad currentGamepad, Gamepad previousGamepad) {
        updateTelemetry("Test");
        if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {
            R.intakeWrist2.setPosition(0.3);
        } else if (currentGamepad.dpad_down && !previousGamepad.dpad_down) {
            R.intakeWrist2.setPosition(0.95);
        }
    }

    public void update(){
        switch(extendoState) {
            case EXTEND:
                moveExtendoTo(extend_position);
                break;
            case RETRACT:
                moveExtendoTo(retract_position);
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
        switch(verticalWristState) {
            case GRAB:
                moveVerticalWristTo(verticalWrist_grab_position);
                break;
            case TRANSFER:
                moveVerticalWristTo(verticalWrist_transfer_position);
                break;
        }
        switch(horizontalWristState) {
            case LEFT_RIGHT:
                moveHorizontalWristTo(horizontalWrist_LR_position);
                break;
            case UP_DOWN:
                moveHorizontalWristTo(horizontalWrist_UD_position);
                break;
            case DIAGONAL_LEFT:
                moveHorizontalWristTo(horizontalWrist_diagonalL_position);
                break;
            case DIAGONAL_RIGHT:
                moveHorizontalWristTo(horizontalWrist_diagonalR_position);
                break;
        }
        switch(armState) {
            case TRANSFER:
                moveArmTo(transfer_position);
                break;
            case HOVER:
                moveArmTo(hover_position);
                break;
            case GRAB:
                moveArmTo(grab_position);
                break;
        }
    }
    public void setExtendoState(ExtendoState state){
        extendoState = state;
    }
    public void setClawState(ClawState state){
        clawState = state;
    }
    public void setArmState(ArmState state){
        armState = state;
    }
    public void setVerticalWristState(VerticalWristState state){
        verticalWristState = state;
    }
    public void setHorizontalWristState(HorizontalWristState state){
        horizontalWristState = state;
    }
}

