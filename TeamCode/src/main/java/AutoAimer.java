import android.util.Size;

import com.qualcomm.robotcore.util.ElapsedTime;

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
CURRENTLY DEPRECATED DUE TO LACK OF SOLUTION FOR APRILTAG FLICKERING
1. create new auto aimer, creating vision portal takes time so do it early
2. call start when starting to go
3. call get motor power to use that to turn towards the tag
4. call stop to stop memory leak
 */
public class AutoAimer {
    protected final Telemetry telemetry;
    protected final AprilTagProcessor aprilTag; //any camera here
    protected final VisionPortal vision;
    private final PIDController pid = new PIDController(0.75, 0.009, 0.2);
    private final ElapsedTime lostTargetTimer = new ElapsedTime();

    // X: distance forward from the robot's center
    // Y: distance right and left of the robot's center
    // Z: distance up from the ground
    private final Position cameraPosition = new Position(DistanceUnit.INCH,
            0, -3, 0, 0);
    // (0,0,0) means exactly forward
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            15, -90, 0, 0);

    private AprilTagDetection bestTag;
    private double averageBearingError = 0;

    public AutoAimer(robot r, Telemetry telemetry) {
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

    public void start() {
        pid.reset();
        lostTargetTimer.reset();
    }

    // stop memory leaks on subsequent init
    public void stop() {
        vision.close();
    }

    // calculates a continuous average to smooth bearing results
    public void update() {
        bestTag = getBestDetection();

        telemetry.addData("cameraState", vision.getCameraState());

        if (bestTag != null) {
            final double newBearing = bestTag.ftcPose.bearing;

            final double alpha = Math.min(0.9, Math.max(0.1, Math.abs(newBearing - averageBearingError) / Math.PI));
            averageBearingError = (alpha * newBearing) + (1 - alpha) * averageBearingError;

            lostTargetTimer.reset();
        } else {
            if (lostTargetTimer.seconds() > 0.25) { // lost target for too long, so reset
                averageBearingError = 0;
                pid.reset();
            }
        }
    }

    // use to prevent wasted camera resources
    public void setActive(boolean active) {
        vision.setProcessorEnabled(aprilTag, active);
        if (!active) {
            averageBearingError = 0;
            pid.reset();
        }
    }

    // returns true if a april tag target is detected
    public boolean hasTarget() {
        return bestTag != null;
    }

    // Returns the turn power to minimize bearing error
    public double getTurnPower() {
        return Math.max(Math.min(pid.calculate(0, averageBearingError), 0.7), -0.7);
    }

    // returns average bearing error in radians
    public double getAverageBearingError() {
        return averageBearingError;
    }

    protected AprilTagDetection getBestDetection() {
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