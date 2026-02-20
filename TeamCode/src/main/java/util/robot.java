package util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.List;

/*
 * sigma
 */

public class robot{
    public DcMotorEx intakeLow, intakeHigh, shooter, shooter2;
    public DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;
    //public CRServo claw;
    public Servo gatekeep;
    public WebcamName camera;

    public robot() {

    }

    public robot(HardwareMap hardwareMap) {
        /*
        intakeWrist1 = hardwareMap.get(Servo.class, "intakeWrist1Servo");
        intakeWrist2 = hardwareMap.get(Servo.class, "intakeWrist2Servo");
        extendo = hardwareMap.get(Servo.class, "extendoServo");
        intakeArm = hardwareMap.get(Servo.class, "intakeArmServo");
        intakeClaw = hardwareMap.get(Servo.class, "intakeClawServo");
        specArm = hardwareMap.get(Servo.class, "specArmServo");
        specArm2 = hardwareMap.get(Servo.class, "specArmServo2");
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        liftMotor2 = hardwareMap.get(DcMotorEx.class, "liftMotor2");
        */
        //camera = hardwareMap.get(WebcamName.class, "camera");
        gatekeep = hardwareMap.get(Servo.class, "gatekeepServo");
        intakeLow = hardwareMap.get(DcMotorEx.class, "intakeLow");
        intakeHigh = hardwareMap.get(DcMotorEx.class, "intakeHigh");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter2  = hardwareMap.get(DcMotorEx.class, "shooter2");

        //claw.setPower(0);

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(0.3, 0, 0, 0));

        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeLow.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeLow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeLow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeHigh.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeHigh.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeHigh.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

 /*       motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }*/
    }
}