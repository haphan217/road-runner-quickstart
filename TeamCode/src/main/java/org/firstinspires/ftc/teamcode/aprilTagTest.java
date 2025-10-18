package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.List;

@TeleOp(name = "Limelight Test", group = "test")
public class aprilTagTest extends LinearOpMode {
    double Kp = 0.2;

    public int id;
    public double x;
    public double y;

    public double pow_y;
    public double pow_x;

    public double distance;

    private Limelight3A limelight;

    public DcMotor Front_Right_Drive;
    public DcMotor Front_Left_Drive;
    public DcMotor Back_Right_Drive;
    public DcMotor Back_Left_Drive;

    public TrajectoryActionBuilder road;

    public boolean finished = true;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Front_Right_Drive = hardwareMap.get(DcMotor.class, "right_front_drive");
        Front_Left_Drive = hardwareMap.get(DcMotor.class, "left_front_drive");
        Back_Right_Drive = hardwareMap.get(DcMotor.class, "right_back_drive");
        Back_Left_Drive = hardwareMap.get(DcMotor.class, "left_back_drive");

        Front_Left_Drive.setDirection(DcMotorSimple.Direction.REVERSE);
        Back_Left_Drive.setDirection(DcMotorSimple.Direction.REVERSE);
        Front_Right_Drive.setDirection(DcMotorSimple.Direction.REVERSE);

        // Start the camera stream on Dashboard
        limelight.start();
        FtcDashboard.getInstance().startCameraStream(limelight, 0);
        limelight.pipelineSwitch(0);

        telemetry.addData(">", "Robot Ready. Press Play.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if(finished == true) {
                LLResult result = limelight.getLatestResult();

                if (result != null && result.isValid()) {
                    List<FiducialResult> fiducials = result.getFiducialResults();
                    for (FiducialResult fiducial : fiducials) {
                        id = fiducial.getFiducialId();
                        x = fiducial.getRobotPoseFieldSpace().getPosition().x;
                        y = fiducial.getRobotPoseFieldSpace().getPosition().y;
                        if (id == 21) {
                            finished = false;
                            break;
                        }
                    }
                    telemetry.addData("x", x);
                    telemetry.addData("y", y);
                    telemetry.addData("id", id);
                    Pose2d beginPose = new Pose2d(x, y, Math.toRadians(180));
                    MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
                    road = drive.actionBuilder(beginPose)
                            .strafeTo(TriangleBorder.GetNearstPos(new Vector2d(x, y)));
                    Actions.runBlocking(road.build());
                } else {
                    telemetry.addLine("No AprilTags detected.");
                }

                telemetry.update();
            }
        }
    }
}
