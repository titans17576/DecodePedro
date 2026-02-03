import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import util.robot;

public class intakeFSM {
    public enum LowIntakeState{
        ON, OFF, REVERSE
    }
    public enum HighIntakeState{
        ON, OFF, REVERSE, STALL
    }
    public enum GatekeepState{
        ON, OFF
    }


    // LiftState instance variable

    robot R;
    Telemetry telemetry;
    LowIntakeState lowIntakeState;
    HighIntakeState highIntakeState;
    GatekeepState gatekeepState;
    final double lowIntakeOn_power = 1;
    final double lowIntakeOff_power = 0;
    final double lowIntakeReverse_power = -1;
    final double highIntakeOn_velocity = 2500;
    final double highIntakeOff_velocity = 0;
    final double highIntakeStall_velocity = 100;
    final double highIntakeReverse_velocity = -2500;
    final double gatekeepOnPosition = 0.26;
    final double gatekeepOffPosition = 0.42;



    // Import opmode variables when instance is created
    public intakeFSM(robot Robot, Telemetry t) {
        this(Robot, t, LowIntakeState.OFF, HighIntakeState.OFF, GatekeepState.ON);
    }
    public intakeFSM(robot Robot, Telemetry t, LowIntakeState lI, HighIntakeState hI, GatekeepState gS) {
        R = Robot;
        telemetry = t;
        lowIntakeState = lI;
        highIntakeState = hI;
        gatekeepState = gS;
    }
    public void initialize() {
    }

    // Method to move to a targeted position
    private void powerLowIntake(Double power) {R.intakeLow.setPower(power);}
    private void powerHighIntake(Double velocity) {R.intakeHigh.setVelocity(velocity);}
    private void setGatekeepPosition(Double position) {R.gatekeep.setPosition(position);}


    // Update method for teleop implementation
    public void teleopUpdate(Gamepad currentGamepad, Gamepad previousGamepad) {

        switch (lowIntakeState) {
            case ON:
                // State inputs
                if (currentGamepad.x && !previousGamepad.x) {
                    setLowIntakeState(LowIntakeState.OFF);
                } else if (currentGamepad.y && !previousGamepad.y) {
                    setLowIntakeState(LowIntakeState.REVERSE);
                } else if (currentGamepad.right_stick_button && !previousGamepad.right_stick_button) {
                    setLowIntakeState(LowIntakeState.OFF);
                }
                break;
            case OFF:
                if (currentGamepad.x && !previousGamepad.x) {
                    setLowIntakeState(LowIntakeState.ON);
                } else if (currentGamepad.y && !previousGamepad.y) {
                    setLowIntakeState(LowIntakeState.REVERSE);
                } else if (currentGamepad.right_stick_button && !previousGamepad.right_stick_button) {
                    setLowIntakeState(LowIntakeState.ON);
                }
                break;
            case REVERSE:
                if (currentGamepad.x && !previousGamepad.x) {
                    setLowIntakeState(LowIntakeState.OFF);
                }
                break;
        }
        switch (highIntakeState) {
            case ON:
                // State inputs
                if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {
                    setHighIntakeState(HighIntakeState.OFF);
                } else if (currentGamepad.left_bumper && !previousGamepad.left_bumper) {
                    setHighIntakeState(HighIntakeState.REVERSE);
                } else if (currentGamepad.right_stick_button && !previousGamepad.right_stick_button) {
                    setHighIntakeState(HighIntakeState.OFF);
                }
                break;
            case OFF:
                if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {
                    setHighIntakeState(HighIntakeState.ON);
                } else if (currentGamepad.left_bumper && !previousGamepad.left_bumper) {
                    setHighIntakeState(HighIntakeState.REVERSE);
                } else if (currentGamepad.right_stick_button && !previousGamepad.right_stick_button) {
                    setHighIntakeState(HighIntakeState.ON);
                }
                break;
            case REVERSE:
                if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {
                    setHighIntakeState(HighIntakeState.OFF);
                }
                break;
        }
        switch (gatekeepState) {
            case ON:
                // State inputs
                if (currentGamepad.right_bumper && !previousGamepad.right_bumper) {
                    setGatekeepState(GatekeepState.OFF);
                }
                break;
            case OFF:
                if (currentGamepad.right_bumper && !previousGamepad.right_bumper) {
                    setGatekeepState(GatekeepState.ON);
                }
                break;
        }
        update();
    }

    public void update(){
        switch(lowIntakeState) {
            case ON:
                powerLowIntake(lowIntakeOn_power);
                break;
            case OFF:
                powerLowIntake(lowIntakeOff_power);
                break;
            case REVERSE:
                powerLowIntake(lowIntakeReverse_power);
                break;
        }
        switch(highIntakeState) {
            case ON:
                powerHighIntake(highIntakeOn_velocity);
                break;
            case OFF:
                powerHighIntake(highIntakeOff_velocity);
                break;
            case REVERSE:
                powerHighIntake(highIntakeReverse_velocity);
                break;
            case STALL:
                powerHighIntake(highIntakeStall_velocity);
                break;
        }
        switch(gatekeepState) {
            case ON:
                setGatekeepPosition(gatekeepOnPosition);
                break;
            case OFF:
                setGatekeepPosition(gatekeepOffPosition);
                break;
        }
    }
    public void setLowIntakeState(LowIntakeState state){
        lowIntakeState = state;
    }
    public void setHighIntakeState(HighIntakeState state){
        highIntakeState = state;
    }
    public void setGatekeepState(GatekeepState state){
        gatekeepState = state;
    }

}

