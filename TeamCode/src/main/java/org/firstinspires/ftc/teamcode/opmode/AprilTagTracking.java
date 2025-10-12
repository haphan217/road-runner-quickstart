package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


import java.util.List;
@Autonomous
public class AprilTagTracking extends LinearOpMode {
    private Limelight3A limelight;
    public DcMotor Front_Right_Drive;
    public DcMotor Front_Left_Drive;
    public DcMotor Back_Right_Drive;
    public DcMotor Back_Left_Drive;
    double kP = 0.05;
    double kD = 0.001;
    ElapsedTime timer = new ElapsedTime();
    private double lastTx = 0;


    double pidController(double tx)
    {
        double derivative = (tx - lastTx) / (timer.seconds());
        lastTx = tx;
        timer.reset();
        return tx * kP + derivative * kD;
    }

    @Override
    public void runOpMode() throws InterruptedException
    {

        Front_Right_Drive = hardwareMap.get(DcMotor.class, "right_front_drive");
        Front_Left_Drive = hardwareMap.get(DcMotor.class, "left_front_drive");
        Back_Right_Drive = hardwareMap.get(DcMotor.class, "right_back_drive");
        Back_Left_Drive = hardwareMap.get(DcMotor.class, "left_back_drive");

        Back_Right_Drive.setDirection(DcMotorSimple.Direction.REVERSE);


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        FtcDashboard.getInstance().startCameraStream(limelight, 0);

        limelight.start();
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            double tx= result.getTx();
            double rotation = pidController(tx);

            telemetry.addData("tx", tx);
            telemetry.addData("rotation", rotation);


            if(Math.abs(tx) > 3) {
                Front_Left_Drive.setPower(rotation);
                Back_Left_Drive.setPower(rotation);
                Front_Right_Drive.setPower(-rotation);
                Back_Right_Drive.setPower(-rotation);
            } else {
                Front_Left_Drive.setPower(0);
                Back_Left_Drive.setPower(0);
                Front_Right_Drive.setPower(0);
                Back_Right_Drive.setPower(0);
            }

            telemetry.update();

//            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
//            for (LLResultTypes.FiducialResult fiducial : fiducials) {
//                int id = fiducial.getFiducialId(); // The ID number of the fiducial
//                double tx = fiducial.getTargetXDegrees(); // Where it is (left-right)
//                double ty = fiducial.getTargetYDegrees(); // Where it is (up-down)
//                double rotation = pidController(tx);
//
//                telemetry.addData("tx", tx);
//                telemetry.addData("rotation", rotation);
//                telemetry.addData("id", id);
//                telemetry.update();


//                Front_Left_Drive.setPower(rotation*0.3);
//                Back_Left_Drive.setPower(rotation*0.3);
//                Front_Right_Drive.setPower(rotation*-0.3);
//                Back_Right_Drive.setPower(rotation*-0.3);

//            }


        }
        limelight.stop();
    }


}
