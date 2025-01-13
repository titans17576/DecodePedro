import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import util.robot;

public class Auto {
    public Follower follower;
    public Telemetry telemetry;

    private robot R;
    public boolean actionBusy;

    private liftFSM LiftFSM;
    private clawFSM ClawFSM;

    public Timer transferTimer = new Timer();
    public int transferState = -1;
    public Auto(robot Robot, Telemetry telemetry, Follower follower) {
        ClawFSM = new clawFSM(Robot, telemetry);
        LiftFSM = new liftFSM(Robot, telemetry);


        this.follower = follower;
        this.telemetry = telemetry;

        
        init();
    }

    public void init() {
        LiftFSM.initialize();
    }
    public void start(){

    }
    public void update(){
        follower.update();
        transfer(); 
    }
    public void transfer(){

    }
    public void setTransferState(int x) {
        transferState = x;
    }

    public void startTransfer() {
        if (actionNotBusy()) {
            setTransferState(1);
        }
    }
    public boolean actionNotBusy() {
        return !actionBusy;
    }

    public boolean notBusy() {
        return (!follower.isBusy() && actionNotBusy());
    }
}