import android.util.Size;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

import pedroPathing.constants.Constants;
import util.robot;

/*
1. create new auto aimer
2. call start when starting to go
3. call update with whether it should be active or not, and don't send conflicting commands to the follower while active
4. it will turn towards the april tag while also localizing the position of the robot
 */
public class AutoAimer {
    private final boolean SHOULD_TURN_TO_POINT_WHEN_ACTIVE = false;// temporarily dissabled
    private final robot r;
    private final Telemetry t;
    private AprilTagProcessor aprilTag; //any camera here
    private VisionPortal vision;
    private final Follower follower;
    private boolean following = false;

    //Put the target location here. default is BLUE GOAL, but this should be immediately changed based on current april tag
    private Pose targetLocation = new Pose(0, 144);

    private final Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    public AutoAimer(robot r, Follower follower, Telemetry t) {
        this.r = r;
        this.t = t;
        this.follower = follower;
    }

    public void start() {
        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera.
        builder.setCamera(r.camera);

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(1280, 720));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        vision = builder.build();
    }

    public void update(boolean isActive) {
        vision.setProcessorEnabled(aprilTag, isActive);
        if (!isActive) {
            // vision.stopStreaming(); documentation says that doing this takes a few seconds, so commented it out
            return;
        } else {
            // vision.resumeStreaming(); documentation says that doing this takes a few seconds, so commented it out
        }

        //This uses the aprilTag to relocalize your robot
        //You can also create a custom AprilTag fusion Localizer for the follower if you want to use this by default for all your autos
        Pose newCamPose = getRobotPoseFromCamera(); // can return null when no april tags detected
        if (newCamPose != null) follower.setPose(newCamPose);

        //if you're not using limelight you can follow the same steps: build an offset pose, put your heading offset, and generate a path etc
        if (!following && SHOULD_TURN_TO_POINT_WHEN_ACTIVE) {
            follower.followPath(
                    follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), targetLocation))
                            .setLinearHeadingInterpolation(follower.getHeading(), targetLocation.minus(follower.getPose()).getAsVector().getTheta())
                            .build()
            );
        }

        if (following && !follower.isBusy()) following = false;
    }

    // get robot pos if possible, and set target position if visible as a side effect
    private Pose getRobotPoseFromCamera() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        Pose robotPose = null;
        Pose targetPose = null;

        for (AprilTagDetection detection : currentDetections) {
            robotPose = new Pose(
                    detection.robotPose.getPosition().x,
                    detection.robotPose.getPosition().y,
                    detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES),
                    FTCCoordinates.INSTANCE
            ).getAsCoordinateSystem(PedroCoordinates.INSTANCE);

            t.addLine(String.format("\n==== (ID %d)", detection.id));
            t.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                    detection.robotPose.getPosition().x,
                    detection.robotPose.getPosition().y,
                    detection.robotPose.getPosition().z));

            t.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                    detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                    detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                    detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));

            if (!(detection.id == 20 || detection.id == 24)) continue; // only look at the goal tags to actually point towards

            targetPose = new Pose(
                    detection.ftcPose.x,
                    detection.ftcPose.y,
                    0,
                    FTCCoordinates.INSTANCE
            ).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
        }

        if (targetPose != null) targetLocation = targetPose;

        return robotPose;
    }
}