import android.util.Size;

import androidx.annotation.NonNull;

import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

import pedroPathing.constants.Constants;
import util.robot;

/*
 * create new auto aimer, creating vision portal takes time so do it early
 * if only using for localization, just call update, not having to call get turn power
 */
public class VisionController {
    private final Telemetry telemetry;
    private final AprilTagProcessor aprilTag; //any camera here
    private final VisionPortal vision;
    private final Follower follower;
    private final ElapsedTime lastDetectionTimer = new ElapsedTime();

    // X: distance forward from the robot's center
    // Y: distance right and left of the robot's center
    // Z: distance up from the ground
    private final Position cameraPosition = new Position(DistanceUnit.INCH,
            1, 3, 8, 0);
    // (0,0,0) means exactly forward
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90 + 15, 0, 0);

    private PIDFController pidController = new PIDFController(Constants.followerConstants.coefficientsHeadingPIDF);
    private Pose targetPose = new Pose(0, 144);
    private ElapsedTime timer = new ElapsedTime();

    public VisionController(@NonNull robot r, Follower follower, Telemetry telemetry) {
        this.follower = follower;
        this.telemetry = telemetry;

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .setCameraPose(cameraPosition, cameraOrientation)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                .setLensIntrinsics(1004.4, 1004.4, 640.0, 360.0) // Values for ardu cam
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
        builder.setCameraResolution(new Size(1280, 720)); // this should be lowered to reduce bandwith

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

    // stop memory leaks on subsequent init
    public void stop() {
        vision.close();
    }

    public void setActive(boolean state) {
        if (!state) pidController.reset();
    }

    // calculates target pose if available
    public void update() {
        AprilTagDetection bestTag = getBestDetection();

        telemetry.addData("camState", vision.getCameraState());
        telemetry.addData("robotPose", follower.getPose());
        telemetry.addData("timeSinceLocalized", timer.seconds());

        if (bestTag != null && bestTag.robotPose != null) {
            timer.reset();
            Pose3D robotPose = bestTag.robotPose;
            Pose rPose = new Pose(
                    robotPose.getPosition().x,
                    robotPose.getPosition().y,
                    robotPose.getOrientation().getYaw(AngleUnit.RADIANS)  + (Math.PI / 2),
                    FTCCoordinates.INSTANCE
            ).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
            follower.setPose(rPose);
        }
    }

    // Returns the turn power to minimize bearing error
    public double getTurnPower() {
        if (targetPose == null) return 0;

        Pose currentPose = follower.getPose();

        double targetHeading = Math.atan2(
                targetPose.getY() - currentPose.getY(),
                targetPose.getX() - currentPose.getX()
        );

        double angleError = AngleUnit.normalizeRadians(targetHeading - currentPose.getHeading());

        pidController.updateError(angleError);
        return Math.min(Math.max(pidController.run(), -0.7), 0.7);
    }

    private AprilTagDetection getBestDetection() {
        List<AprilTagDetection> detections = aprilTag.getFreshDetections();
        if (detections == null || detections.isEmpty()) return null; // no fresh data

        AprilTagDetection best = null;
        double minBearing = Double.MAX_VALUE;

        for (AprilTagDetection detection : detections) {
            if (detection.decisionMargin < 40) continue;

            if (Math.abs(detection.ftcPose.bearing) < minBearing) {
                minBearing = Math.abs(detection.ftcPose.bearing);
                best = detection;
            }
        }
        return best;
    }
}