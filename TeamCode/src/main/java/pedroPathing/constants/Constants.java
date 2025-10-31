package pedroPathing.constants;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

  public static FollowerConstants followerConstants = new FollowerConstants()
    .mass(12)
    .forwardZeroPowerAcceleration(-26.8997)
    .lateralZeroPowerAcceleration(-58.1159)
    .useSecondaryTranslationalPIDF(false)
    .useSecondaryHeadingPIDF(false)
    .useSecondaryDrivePIDF(false)
    .centripetalScaling(0.0002)
    .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.04, 0))
    .headingPIDFCoefficients(new PIDFCoefficients(2, 0, 0.1, 0))
    .drivePIDFCoefficients(
      new FilteredPIDFCoefficients(0.013, 0, 0.00003, 0.6, 0)
    );

  public static MecanumConstants driveConstants = new MecanumConstants()
    .leftFrontMotorName("leftFront")
    .leftRearMotorName("leftRear")
    .rightFrontMotorName("rightFront")
    .rightRearMotorName("rightRear")
    .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
    .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
    .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
    .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
    .xVelocity(78.0501)
    .yVelocity(62.2552)
    .useBrakeModeInTeleOp(true);

  public static PinpointConstants localizerConstants = new PinpointConstants()
    .forwardPodY(6)
    .strafePodX(-3.75)
    .distanceUnit(DistanceUnit.INCH)
    .hardwareMapName("pinpoint")
    .yawScalar(1.0)
    .encoderResolution(
      GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
    )
    .customEncoderResolution(13.26291192)
    .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
    .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

  public static PathConstraints pathConstraints = new PathConstraints(
    0.99,
    100,
    2,
    1
  );

  public static Follower createFollower(HardwareMap hardwareMap) {
    return new FollowerBuilder(followerConstants, hardwareMap)
      .mecanumDrivetrain(driveConstants)
      .pinpointLocalizer(localizerConstants)
      .pathConstraints(pathConstraints)
      .build();
  }
}
