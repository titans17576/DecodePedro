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
    .mass(9.097)
    .forwardZeroPowerAcceleration(-53.7478)
    .lateralZeroPowerAcceleration(-60.6648)
    .useSecondaryTranslationalPIDF(false)
    .useSecondaryHeadingPIDF(true)
    .useSecondaryDrivePIDF(true)
    .centripetalScaling(0.0003)
    .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0.03))
    .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0, 0.03))
    .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1, 0,0.01, 0.03))
    .drivePIDFCoefficients(
      new FilteredPIDFCoefficients(0.02, 0, 0.004, 0.6, 0.03)
    )
    .secondaryDrivePIDFCoefficients(
      new FilteredPIDFCoefficients(0.02, 0, 0.005, 0.6, 0.01)
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
    .xVelocity(81.0484)
    .yVelocity(62.9697)
    .useBrakeModeInTeleOp(true);

  public static PinpointConstants localizerConstants = new PinpointConstants()
    .forwardPodY(-3)
    .strafePodX(-3.75)
    .distanceUnit(DistanceUnit.INCH)
    .hardwareMapName("pinpoint")
    .yawScalar(1.0)
    .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
    .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
    .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

  public static PathConstraints pathConstraints = new PathConstraints(
    0.995,
    100,
    1.3,
    2
  );
// brakingStrength was 1.3, brakingStart was 2
  public static Follower createFollower(HardwareMap hardwareMap) {
    return new FollowerBuilder(followerConstants, hardwareMap)
      .mecanumDrivetrain(driveConstants)
      .pinpointLocalizer(localizerConstants)
      .pathConstraints(pathConstraints)
      .build();
  }
}
