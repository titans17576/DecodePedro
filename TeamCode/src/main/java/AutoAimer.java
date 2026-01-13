import android.util.Size;

import com.pedropathing.control.LowPassFilter;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

import util.robot;

/*
1. create new auto aimer, creating vision portal takes time so do it early
2. call start when starting to go
3. call get motor power to use that to turn towards the tag
4. call stop to stop memory leak
 */
public class AutoAimer {
    private final Telemetry telemetry;
    private final AprilTagProcessor aprilTag; //any camera here
    private final VisionPortal vision;
    private final Follower follower;

    private final LowPassFilter xFilter = new LowPassFilter(0.2);
    private final LowPassFilter yFilter = new LowPassFilter(0.2);
    private final LowPassFilter headingFilter = new LowPassFilter(0.2);

    // X: distance forward from the robot's center
    // Y: distance right and left of the robot's center
    // Z: distance up from the ground
    private final Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    // (0,0,0) means exactly forward
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, 0, 0, 0);

    private Pose targetPose;

    public AutoAimer(robot r, Follower follower, Telemetry telemetry) {
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
                .setLensIntrinsics(502.2, 502.2, 320.0, 180.0) // Values for ardu cam
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
        builder.setCameraResolution(new Size(640, 360)); // this should be lowered to reduce bandwith

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

    // calculates a continuous average to smooth bearing results
    public void update() {
        AprilTagDetection bestTag = getBestDetection();

        if (bestTag != null && bestTag.metadata != null) {
            Pose rawPose = new Pose(
                    bestTag.robotPose.getPosition().x,
                    bestTag.robotPose.getPosition().y,
                    bestTag.robotPose.getOrientation().getYaw(AngleUnit.RADIANS)
            ).getAsCoordinateSystem(PedroCoordinates.INSTANCE);

            double distanceToNewDetection = rawPose.distanceFrom(follower.getPose());
            if (distanceToNewDetection > 15) {
                // other two values not used in LowPassFilter
                xFilter.reset(rawPose.getX(), 0, 0);
                yFilter.reset(rawPose.getY(), 0, 0);
                headingFilter.reset(rawPose.getHeading(), 0, 0);
            } else {
                // updateProjection not used but still required for all filters??
                xFilter.update(rawPose.getX(), 0);
                yFilter.update(rawPose.getY(), 0);
                headingFilter.update(rawPose.getHeading(), 0);
            }

            follower.setPose(new Pose(xFilter.getState(), yFilter.getState(), headingFilter.getState()));

            targetPose = new Pose(
                    bestTag.metadata.fieldPosition.get(0),
                    bestTag.metadata.fieldPosition.get(1)
            ).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
        }
    }

    // use to prevent wasted camera resources
    public void setActive(boolean active) {
        vision.setProcessorEnabled(aprilTag, active);
    }

    // Returns the turn power to minimize bearing error
    public double getTurnPower() {
        if (targetPose == null) return 0;

        Pose currentPose = follower.getPose();

        double angleToTarget = Math.atan2(
                targetPose.getY() - currentPose.getY(),
                targetPose.getX() - currentPose.getX()
        );

        double angleError = angleToTarget - currentPose.getHeading();
        while (angleError > Math.PI) angleError -= 2 * Math.PI;
        while (angleError < -Math.PI) angleError += 2 * Math.PI;
        if (Math.abs(angleError) < 0.02) angleError = 0;

        double power = angleError * 0.5;
        return Math.max(-1.0, Math.min(1.0, power));
    }

    private AprilTagDetection getBestDetection() {
        List<AprilTagDetection> detections = aprilTag.getFreshDetections();
        if (detections == null || detections.isEmpty()) return null; // no fresh data

        AprilTagDetection best = null;
        double minBearing = Double.MAX_VALUE;

        for (AprilTagDetection detection : detections) {
            if (detection.decisionMargin < 25) continue;

            // id 20, 24 are for goals
            if (detection.id == 20 || detection.id == 24) {
                if (Math.abs(detection.ftcPose.bearing) < minBearing) {
                    minBearing = Math.abs(detection.ftcPose.bearing);
                    best = detection;
                }
            }
        }
        return best;
    }
}