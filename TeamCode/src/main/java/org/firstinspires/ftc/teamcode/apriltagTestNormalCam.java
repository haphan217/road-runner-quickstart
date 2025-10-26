package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

@TeleOp(name = "camera apriltag")
public class apriltagTestNormalCam extends LinearOpMode {
    private static double fieldSize = 144; // inches
    private static double goalHeight = 38.75; // inches
    private static double obeliskHeight = 23; // inches
    private static double goalAprilTagHeightOffset = 9.25; // inches
    private static double aprilTagSize = 8.125; // inches
    private static double goalAprilTagCenterZ = goalHeight - goalAprilTagHeightOffset;
    private static double obeliskAprilTagCenterZ = obeliskHeight - aprilTagSize/2;
    private static final boolean USE_WEBCAM = true;
    private Vector3d positionIdObelisk = new Vector3d(-fieldSize/2, 0, obeliskAprilTagCenterZ);
    private Vector3d positionIdRed = new Vector3d(-58.3727, 55.6425, goalAprilTagCenterZ);
    private Vector3d positionIdBlue = new Vector3d(-58.3727, -55.6425, goalAprilTagCenterZ);
    private double degressOfpositionIdObelisk = 0;
    private double degressOfpositionIdRed = 45;
    private double degressOfpositionIdBlue = -45;
    private Vector3d positionRobot = new Vector3d(0, 0, 0);
    private final Position cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Telemetry dashboardTelemetry = dashboard.getTelemetry();
    public TrajectoryActionBuilder road;

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
            dashboardTelemetry.update();

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
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTag)
                .build();
        FtcDashboard.getInstance().startCameraStream(visionPortal, 0);
    }
    @SuppressLint("DefaultLocale")
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                dashboardTelemetry.addData("# AprilTags Detected", currentDetections.size());
                dashboardTelemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
//                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.robotPose.getPosition().x, detection.robotPose.getPosition().y, detection.robotPose.getPosition().z));
//                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES), detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES), detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
                dashboardTelemetry.addLine(String.format("ROBOT XYZ %6.1f %6.1f %6.1f  (inch)", detection.robotPose.getPosition().x, detection.robotPose.getPosition().y, detection.robotPose.getPosition().z));
                dashboardTelemetry.addLine(String.format("FTC XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
//                dashboardTelemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES), detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES), detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));

                if(detection.id == 21 || detection.id ==22 || detection.id ==23) {
                    double x = detection.robotPose.getPosition().x;
                    double y = Math.abs(detection.robotPose.getPosition().y)-72;
                    positionRobot.set(new Vector3d(x, y, 0));
                    telemetry.addLine(String.format("OBELISK XYZ Robo %6.1f %6.1f (inch)", x, y));
                    dashboardTelemetry.addLine(String.format("OBELISK XYZ Robo %6.1f %6.1f (inch)", x, y));
                    Pose2d beginPose = new Pose2d(y, x, Math.toRadians(180));
                    MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
                    road = drive.actionBuilder(beginPose)
                            .strafeTo(TriangleBorder.GetNearstPos(new Vector2d(y, x)));
                    Actions.runBlocking(road.build());
                } else if(detection.id == 24){
                    double x = positionIdRed.x + (detection.robotPose.getPosition().z*Math.sin(degressOfpositionIdRed));
                    double y = positionIdRed.y + (detection.robotPose.getPosition().z*Math.cos(degressOfpositionIdRed));
                    double distanceShot = Math.sqrt(Math.pow(y, 2) + Math.pow(x, 2));
                    double degressShot = Math.asin(y/distanceShot);
                    double powShot = distanceShot/90; // giả sử khi pow = 1 thì nó sẽ bắn xa được 90 inch
                    positionRobot.set(new Vector3d(x, y, 0));
                    telemetry.addLine(String.format("RED GOAL XYZ Robo %6.1f %6.1f %6.1f (inch)", x, y));
                    telemetry.addLine(String.format("Pow Degress Robo %6.2f %6.2f %6.2f (inch)", distanceShot, degressShot, powShot));
                    dashboardTelemetry.addLine(String.format("RED GOAL XYZ Robo %6.1f %6.1f %6.1f (inch)", x, y));
                    dashboardTelemetry.addLine(String.format("Pow Degress Robo %6.2f %6.2f %6.2f (inch)", distanceShot, degressShot, powShot));
                } else if(detection.id == 20){
                    double x = positionIdBlue.x + (detection.robotPose.getPosition().z*Math.sin(degressOfpositionIdBlue));
                    double y = positionIdBlue.y + (detection.robotPose.getPosition().z*Math.cos(degressOfpositionIdBlue));
                    double distanceShot = Math.sqrt(Math.pow(y, 2) + Math.pow(x, 2));
                    double degressShot = Math.asin(y/distanceShot);
                    double powShot = distanceShot/90; // giả sử khi pow = 1 thì nó sẽ bắn xa được 90 inch
                    positionRobot.set(new Vector3d(x, y, 0));
                    telemetry.addLine(String.format("BLUE GOAL XYZ Robo %6.1f %6.1f (inch)", x, y));
                    telemetry.addLine(String.format("Pow Degress Robo %6.2f %6.2f %6.2f (inch)", distanceShot, degressShot, powShot));
                    dashboardTelemetry.addLine(String.format("BLUE GOAL XYZ Robo %6.1f %6.1f (inch)", x, y));
                    dashboardTelemetry.addLine(String.format("Pow Degress Robo %6.2f %6.2f %6.2f (inch)", distanceShot, degressShot, powShot));
                }
            } else {
                telemetry.addLine(("\n==== NO TAG DETECTED"));
            }
        }
    }
}
