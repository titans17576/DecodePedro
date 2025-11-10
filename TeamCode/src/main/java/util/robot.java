package util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

/*
 * sigma
 */

public class robot{
    public DcMotorEx  liftMotor, intakeLow, intakeHigh, shooter;
    public DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;
    //public CRServo claw;
    public Servo claw;

    public robot() {

    }

    public robot(HardwareMap hardwareMap) {
        /*
        intakeWrist1 = hardwareMap.get(Servo.class, "intakeWrist1Servo");
        intakeWrist2 = hardwareMap.get(Servo.class, "intakeWrist2Servo");
        claw = hardwareMap.get(Servo.class, "clawServo");
        extendo = hardwareMap.get(Servo.class, "extendoServo");
        intakeArm = hardwareMap.get(Servo.class, "intakeArmServo");
        intakeClaw = hardwareMap.get(Servo.class, "intakeClawServo");
        specArm = hardwareMap.get(Servo.class, "specArmServo");
        specArm2 = hardwareMap.get(Servo.class, "specArmServo2");
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        liftMotor2 = hardwareMap.get(DcMotorEx.class, "liftMotor2");
        */
        intakeLow = hardwareMap.get(DcMotorEx.class, "intakeLow");
        intakeHigh = hardwareMap.get(DcMotorEx.class, "intakeHigh");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        //claw.setPower(0);

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(0.3, 0, 0, 0));

        intakeLow.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeLow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeLow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeHigh.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeHigh.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeHigh.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

 /*       motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }*/
    }
}