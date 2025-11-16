import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import util.robot;

public class intakeFSM {
    public enum LowIntakeState{
        ON,
        OFF
    }
    public enum HighIntakeState{
        ON,
        OFF
    }
    public enum GatekeepState{
        ON,
        OFF
    }


    // LiftState instance variable

    robot R;
    Telemetry telemetry;
    LowIntakeState lowIntakeState;
    HighIntakeState highIntakeState;
    GatekeepState gatekeepState;
    final double lowIntakeOn_power = 1;
    final double lowIntakeOff_power = 0;
    final double highIntakeOn_power = 1;
    final double highIntakeOff_power = 0;
    final double gatekeepOnPosition = 0.3;
    final double gatekeepOffPosition = 0.4;



    // Import opmode variables when instance is created
    public intakeFSM(robot Robot, Telemetry t) {
        this(Robot, t, LowIntakeState.OFF, HighIntakeState.OFF, GatekeepState.OFF);
    }
    public intakeFSM(robot Robot, Telemetry t, LowIntakeState lI, HighIntakeState hI, GatekeepState gS) {
        R = Robot;
        telemetry = t;
        lowIntakeState = lI;
        highIntakeState = hI;
        gatekeepState = gS;
    }
    public void initialize() {
        R.intakeLow.setPower(0);
        R.intakeHigh.setPower(0);
    }

    // Method to move to a targeted position
    private void powerLowIntake(Double power) {R.intakeLow.setPower(power);}
    private void powerHighIntake(Double power) {R.intakeHigh.setPower(power);}
    private void setGatekeepPosition(Double position) {R.gatekeep.setPosition(position);}


    // Update method for teleop implementation
    public void teleopUpdate(Gamepad currentGamepad, Gamepad previousGamepad) {

        switch (lowIntakeState) {
            case ON:
                // State inputs
                if (currentGamepad.x && !previousGamepad.x) {
                    setLowIntakeState(LowIntakeState.OFF);
                }
                break;
            case OFF:
                if (currentGamepad.x && !previousGamepad.x) {
                    setLowIntakeState(LowIntakeState.ON);
                }
                break;
        }
        switch (highIntakeState) {
            case ON:
                // State inputs
                if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {
                    setHighIntakeState(HighIntakeState.OFF);
                }
                break;
            case OFF:
                if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {
                    setHighIntakeState(HighIntakeState.ON);
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
        }
        switch(highIntakeState) {
            case ON:
                powerHighIntake(highIntakeOn_power);
                break;
            case OFF:
                powerHighIntake(highIntakeOff_power);
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

