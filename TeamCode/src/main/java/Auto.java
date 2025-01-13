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
        switch(transferState){
            case 1:
                actionBusy = true;
                ClawFSM.setState(ClawFSM.ClawState.CLOSED);
                transferTimer.resetTimer();
                setTransferState(2);
                break;
            case 2:
                if(transferTimer.getElapsedTimeSeconds() > 1.5){
                    LiftFSM.setState(LiftFSM.LiftState.HIGH);
                    transferTimer.resetTimer();
                    setTransferState(3);
                }

                break;
            case 3:
                if(transferTimer.getElapsedTimeSeconds() > 1){
                    ClawFSM.setState(ClawFSM.ClawState.OPEN);
                    actionBusy = false;
                    setTransferState(-1);
                }
                break;


        }
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