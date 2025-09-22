package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


import java.util.List;

public class AprilTagTracking extends LinearOpMode {
    private Limelight3A limelight;
    public DcMotor motor;

    double limelightAimProportional(double tx)
    {
        double kP = .015;
        double targetingAngularVelocity = tx * kP;
        targetingAngularVelocity *= -1.0;
        return targetingAngularVelocity;
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        motor = hardwareMap.get(DcMotor.class, "motor");

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
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                int id = fiducial.getFiducialId(); // The ID number of the fiducial
                double tx = fiducial.getTargetXDegrees(); // Where it is (left-right)
                double ty = fiducial.getTargetYDegrees(); // Where it is (up-down)
                double rotation = limelightAimProportional(tx);

                telemetry.addData("tx", tx);
                telemetry.addData("rotation", rotation);
                telemetry.addData("ty", ty);
                telemetry.update();

                motor.setPower(rotation);

            }


        }
        limelight.stop();
    }


}
