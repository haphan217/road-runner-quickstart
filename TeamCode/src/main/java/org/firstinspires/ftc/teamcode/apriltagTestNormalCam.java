package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class apriltagTestNormalCam extends LinearOpMode {
    AprilTagProcessor apriltag;
    VisionPortal camera;
    AprilTagProcessor myAprilTagProcessor;
    List<AprilTagDetection> myAprilTagDetections;  // list of all detections
    int myAprilTagIdCode;
    @Override
    public void runOpMode() throws InterruptedException {
        apriltag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        camera = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(apriltag)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        camera.setProcessorEnabled(apriltag, true);
        FtcDashboard.getInstance().startCameraStream(camera, 0);

        while (opModeIsActive()){
            myAprilTagDetections = myAprilTagProcessor.getDetections();

            for (AprilTagDetection myAprilTagDetection : myAprilTagDetections) {

                if (myAprilTagDetection.metadata != null) {  // This check for non-null Metadata is not needed for reading only ID code.
                    myAprilTagIdCode = myAprilTagDetection.id;

                    // Now take action based on this tag's ID code, or store info for later action.
                    telemetry.addData("id", myAprilTagIdCode);
                }
            }
            telemetry.update();
        }
    }
}
