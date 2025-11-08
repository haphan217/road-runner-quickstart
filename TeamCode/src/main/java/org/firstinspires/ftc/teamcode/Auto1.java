package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Auto1")
public class Auto1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
                            Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(180));
                            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
                            TrajectoryActionBuilder road = drive.actionBuilder(beginPose)
                                    .strafeTo(new Vector2d(-24, 24));
                            waitForStart();

                            drive.leftFront.setPower(0.5);
                            drive.rightFront.setPower(0.5);
                            drive.leftBack.setPower(0.5);
                            drive.rightBack.setPower(0.5);

                            // cập nhật pose từ localizer
                            drive.localizer.update(); // rất quan trọng
                            sleep(500);

                            drive.leftFront.setPower(0);
                            drive.rightFront.setPower(0);
                            drive.leftBack.setPower(0);
                            drive.rightBack.setPower(0);

                            Actions.runBlocking(road.build());
                            telemetry.addData("x of ROBOT", drive.localizer.getPose().position.x);
                            telemetry.addData("y of ROBOT", drive.localizer.getPose().position.y);
                            telemetry.update();
    }
}
