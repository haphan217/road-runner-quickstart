package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "MotorTest")
public class MotorTest extends LinearOpMode {
    private DcMotor leftMotor;
    private DcMotor rightMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        leftMotor = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightMotor = hardwareMap.get(DcMotor.class, "left_back_drive");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {
            // Example: both motors run at 50% power in opposite directions
            leftMotor.setPower(1);
            rightMotor.setPower(1);

            telemetry.addData("Left Motor Power", leftMotor.getPower());
            telemetry.addData("Right Motor Power", rightMotor.getPower());
            telemetry.update();
        }

        // Stop motors at the end
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
}
