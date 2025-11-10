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


    // LiftState instance variable

    robot R;
    Telemetry telemetry;
    LowIntakeState lowIntakeState;
    HighIntakeState highIntakeState;
    final double lowIntakeOn_power = 1;
    final double lowIntakeOff_power = 0;
    final double highIntakeOn_power = 1;
    final double highIntakeOff_power = 0;



    // Import opmode variables when instance is created
    public intakeFSM(robot Robot, Telemetry t) {
        this(Robot, t, LowIntakeState.OFF, HighIntakeState.OFF);
    }
    public intakeFSM(robot Robot, Telemetry t, LowIntakeState lI, HighIntakeState hI) {
        R = Robot;
        telemetry = t;
        lowIntakeState = lI;
        highIntakeState = hI;
    }
    public void initialize() {
        R.intakeLow.setPower(0);
        R.intakeHigh.setPower(0);
    }

    // Method to move to a targeted position
    private void powerLowIntake(Double power) {R.intakeLow.setPower(power);}
    private void powerHighIntake(Double power) {R.intakeHigh.setPower(power);}

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
        /*if (currentGamepad.y && !previousGamepad.y) {
            setArmState(ArmState.TRANSFER);
            setVerticalWristState(VerticalWristState.TRANSFER);
            setHorizontalWristState(HorizontalWristState.LEFT_RIGHT);
        } else if (currentGamepad.a && !previousGamepad.a) {
            setArmState(ArmState.HOVER);
            setVerticalWristState(VerticalWristState.GRAB);
        } else if (currentGamepad.x && !previousGamepad.x) {
            setArmState(ArmState.GRAB);
            setVerticalWristState(VerticalWristState.GRAB);
        }*/

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
    }
    public void setLowIntakeState(LowIntakeState state){
        lowIntakeState = state;
    }
    public void setHighIntakeState(HighIntakeState state){
        highIntakeState = state;
    }
}

