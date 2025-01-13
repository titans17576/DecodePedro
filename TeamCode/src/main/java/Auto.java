import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import util.robot;

public class Auto {
    public Follower follower;
    public Telemetry telemetry;

    private robot R;

    private liftFSM LiftFSM;
    private clawFSM ClawFSM;
    public Auto(robot Robot, Telemetry telemetry, Follower follower) {
        ClawFSM = new clawFSM(Robot, telemetry);
        LiftFSM = new liftFSM(Robot, telemetry);


        this.follower = follower;
        this.telemetry = telemetry;

        
        init();
    }

    public void init() {

    }
    public void start(){

    }
    public void update(){

    }
    public void transfer(){

    }
}