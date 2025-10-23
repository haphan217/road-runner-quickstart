package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;

import java.util.List;

@TeleOp(name = "camerta apriltag")
public class apriltagTestNormalCam extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;
    private double tagSize = 0.1524;
    private Vector3d positionIdObelisk = new Vector3d(0, 150, 15);
    private Vector3d positionIdRed = new Vector3d(36, 36, 100);
    private Vector3d positionIdBlue = new Vector3d(-36, 36, 100);
    private double degressOfpositionIdObelisk = 0;
    private double degressOfpositionIdRed = 45;
    private double degressOfpositionIdBlue = -45;
    private Vector3d positionRobot = new Vector3d(0, 0, 0);
    private final Position cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {

        initAprilTag();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            telemetryAprilTag();

            // Push telemetry to the Driver Station.
            telemetry.update();

            // Save CPU resources; can resume streaming when needed.
            if (gamepad1.dpad_down) {
                visionPortal.stopStreaming();
            } else if (gamepad1.dpad_up) {
                visionPortal.resumeStreaming();
            }

            // Share the CPU.
            sleep(20);
        }
        visionPortal.close();

    }
    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
        builder.addProcessor(aprilTag);

        visionPortal = builder.build();
        FtcDashboard.getInstance().startCameraStream(visionPortal, 0);
    }
    @SuppressLint("DefaultLocale")
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.robotPose.getPosition().x, detection.robotPose.getPosition().y, detection.robotPose.getPosition().z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES), detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES), detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));

                if(detection.id == 21 || detection.id == 22 || detection.id == 23){
                    double x = positionIdObelisk.x + (detection.robotPose.getPosition().z*Math.sin(degressOfpositionIdObelisk));
                    double y = positionIdObelisk.y + (detection.robotPose.getPosition().z*Math.cos(degressOfpositionIdObelisk));
                    positionRobot.set(new Vector3d(x, y, 0));
                    telemetry.addLine(String.format("XYZ Robo %6.1f %6.1f (inch)", x, y));
                } else if(detection.id == 22){
                    double x = positionIdRed.x + (detection.robotPose.getPosition().z*Math.sin(degressOfpositionIdRed));
                    double y = positionIdRed.y + (detection.robotPose.getPosition().z*Math.cos(degressOfpositionIdRed));
                    double distanceShot = Math.sqrt(Math.pow(y, 2) + Math.pow(x, 2));
                    double degressShot = Math.asin(y/distanceShot);
                    double powShot = distanceShot/90; // giả sử khi pow = 1 thì nó sẽ bắn xa được 90 inch
                    positionRobot.set(new Vector3d(x, y, 0));
                    telemetry.addLine(String.format("XYZ Robo %6.1f %6.1f %6.1f (inch)", x, y));
                    telemetry.addLine(String.format("Pow Degress Robo %6.2f %6.2f %6.2f (inch)", distanceShot, degressShot, powShot));
                } else if(detection.id == 23){
                    double x = positionIdBlue.x + (detection.robotPose.getPosition().z*Math.sin(degressOfpositionIdBlue));
                    double y = positionIdBlue.y + (detection.robotPose.getPosition().z*Math.cos(degressOfpositionIdBlue));
                    double distanceShot = Math.sqrt(Math.pow(y, 2) + Math.pow(x, 2));
                    double degressShot = Math.asin(y/distanceShot);
                    double powShot = distanceShot/90; // giả sử khi pow = 1 thì nó sẽ bắn xa được 90 inch
                    positionRobot.set(new Vector3d(x, y, 0));
                    telemetry.addLine(String.format("XYZ Robo %6.1f %6.1f (inch)", x, y));
                    telemetry.addLine(String.format("Pow Degress Robo %6.2f %6.2f %6.2f (inch)", distanceShot, degressShot, powShot));
                }
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f (pixels)", detection.center.x, detection.center.y));
            }
        }

        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
    }
}
