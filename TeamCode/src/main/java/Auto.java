import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
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
    public Path forwards, backwards;

    public Pose speciman1Pose,speciman2Pose, speciman3Pose, longBack2Pose, longBack3Pose, longBack4Pose, shift2Pose, shift3Pose, shift4Pose, zone2Pose, zone3Pose, zone4Pose, shortBack1Pose;

    public PathChain moveCurve;
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
        LiftFSM.update();
        ClawFSM.update();


        transfer(); 
    }
    public void transfer(){
        switch(transferState){
            case 1:
                actionBusy = true;
                ClawFSM.setState(clawFSM.ClawState.CLOSED);
                transferTimer.resetTimer();
                setTransferState(2);
                break;
            case 2:
                if(transferTimer.getElapsedTimeSeconds() > 1.5){
                    LiftFSM.setState(liftFSM.LiftState.MID);
                    transferTimer.resetTimer();
                    setTransferState(3);
                }
                break;
            case 3:
                if(LiftFSM.actionNotBusy()){
                    LiftFSM.setState(liftFSM.LiftState.LOW);
                    transferTimer.resetTimer();
                    setTransferState(4);
                }
            case 4:
                if(LiftFSM.actionNotBusy()){
                    ClawFSM.setState(clawFSM.ClawState.OPEN);
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
    public void buildPaths(){
        moveCurve = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(9.484, 107.064),
                                new Point(34.440, 107.813),
                                new Point(13.227, 73.373),
                                new Point(39.182, 73.373)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();
        forwards = new Path(new BezierLine(new Point(0,0, Point.CARTESIAN), new Point(40,0, Point.CARTESIAN)));
        forwards.setConstantHeadingInterpolation(0);
        backwards = new Path(new BezierLine(new Point(40,0, Point.CARTESIAN), new Point(0,0, Point.CARTESIAN)));
        backwards.setConstantHeadingInterpolation(0);

    }
}