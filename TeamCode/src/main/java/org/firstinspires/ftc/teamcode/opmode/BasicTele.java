package org.firstinspires.ftc.teamcode.opmode;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
/// ░░░░░░░░░░░░▄▄
/// ░░░░░░░░░░░█░░█
/// ░░░░░░░░░░░█░░█
/// ░░░░░░░░░░█░░░█
/// ░░░░░░░░░█░░░░█
/// ███████▄▄█░░░░░██████▄
/// ▓▓▓▓▓▓█░░░░░░░░░░░░░░█
/// ▓▓▓▓▓▓█░░░░░░░░░░░░░░█
/// ▓▓▓▓▓▓█░░░░░░░░░░░░░░█
/// ▓▓▓▓▓▓█░░░░░░░░░░░░░░█
/// ▓▓▓▓▓▓█░░░░░░░░░░░░░░█
/// ▓▓▓▓▓▓█████░░░░░░░░░█
/// ██████▀░░░░▀▀██████▀
/// ╔═╦╗╔╦╗╔═╦═╦╦╦╦╗╔═╗
/// ║╚╣║║║╚╣╚╣╔╣╔╣║╚╣═╣
/// ╠╗║╚╝║║╠╗║╚╣║║║║║═╣
/// ╚═╩══╩═╩═╩═╩╝╚╩═╩═╝
/// © Bản quyền thuộc về Billy VN
/// © Copyright by Billy VN

//////////////////////////////////
/// Ki hieu code               ///
/// M: DCMotor                 ///
/// S: Servo                   ///
/// CS: Continous Servo        ///
/// T: Timer                   ///
/// I: Int                     ///
/// F: double                  ///
/// Th: thread                 ///
/// B: Boolean                 ///
/// Ds: distance               ///
//////////////////////////////////
@Disabled
@TeleOp(name = "TeleBieuDien", group = "Đội Tuyển Robot First FPT Da Nang VN")
public class BasicTele extends LinearOpMode {
    // 1. Khai báo phần cứng (biến động cơ, servo, cảm biến,...)
    // Can Gat
    public double FNoTakeSamplePos = 1;
    public double FTakeSamplePos = 0.45;
    // Can Gat 3
    public double FCanGat3TakeSamplePos = 0.185;
    public double FCanGat3ReturnHome = 0.4472;
//Kep Duoi
public double FKepNoTakeSample = 0.9739;
    public double FKepTakeSample = 0.9278;
    //Kep Tren
    public double FKepTrenTakeSample = 0.4694;
    public double FKepTrenNoTakeSample = 0.5178;
    //Can Xoay Nho Tren
    public double FCanXoayNhoTrenSnapSampleToBar = 1;
    //Can Thut Tren
    public double FCanThutTrenOut = 1;
    public double FCanThutTrenReturn = 0;
    //Can Xoay Ngang Duoi
    public double FReturnMid = 0.3178;
    public double angleCanXoayNgangDuoi;
    /// Boolean
    //Thanh Truot Duoi
    public boolean BAutoTakeSample = false;
    public boolean BAutoTakeSampleYellow = false;
    public boolean BAutoTakeSampleYellowComplette = true;
    public boolean BCamera = false;
    public boolean BOpenCV = false;
    public boolean BThanhDungauto = false;
    public boolean BThanhDuoiauto = false;
    public boolean BTranfer = false;
    /// Motor
    // drivetrain
    public DcMotor MLeftFrontDrive;
    public DcMotor MRightFrontDrive;
    public DcMotor MLeftBackDrive;
    public DcMotor MRightBackDrive;
    // Intake
    public DcMotor MThanhTruotDuoi;
    // Outtake
    public DcMotor MThanhDung1;
    public DcMotor MThanhDung2;
    public DcMotor MThanhDung3;
    /// Servo
    // Intake
    public Servo SCanGat1;
    public Servo SCanGat2;
    public Servo SCanGat3;
    public Servo SKepDuoi;
    //Outtake
    public Servo SKepTren;
    public Servo SCanXoayTren1;
    public Servo SCanXoayTren2;
    public Servo SCanXoayNhoTren;
    public Servo SCanThutTren;
    public Servo SCanXoayNgangDuoi;
    /// Timer
    public ElapsedTime TCanGat = new ElapsedTime();
    public ElapsedTime TTakeSampleWithClip = new ElapsedTime();
    public ElapsedTime TCanXoayNgang = new ElapsedTime();
    public ElapsedTime TTurnOnCamera = new ElapsedTime();
    public ElapsedTime TCanXoayNgangDuoi = new ElapsedTime();
    public ElapsedTime TCanGat3 = new ElapsedTime();
    public ElapsedTime TCanXoaytren = new ElapsedTime();
    public ElapsedTime TCanThuttren = new ElapsedTime();
    public ElapsedTime TKepTren = new ElapsedTime();
    /// Sensor
    public DistanceSensor Dsdistance;
    //public DistanceSensor DsdistanceUp;
    public ColorSensor ClColor;
    public DistanceSensor DsBackDis;

    public boolean isStartPressed = false;
    double[] hsvCenter;

    // ================== THÊM BIẾN CHẾ ĐỘ Ở ĐÂY ==================
    public enum ColorMode {
        BLUE_ONLY,
        BLUE_AND_YELLOW
    }
    public ColorMode currentMode = ColorMode.BLUE_ONLY;
    public enum BSampleIntake {
        HAS_BLUE,
        HAS_BLUE_OR_YELLOW,
        NONE
    }
    public BSampleIntake currentSample = BSampleIntake.NONE;

    boolean optionPressedLastFrame = false;

    boolean isMovingUp = false;

    boolean BatCamera = false;
    boolean isMovingDown = false;

    public IMU imu;

    boolean isTripleMotorRunning = false;
    int targetPos = 0;
    double targetPower = 0;
    DcMotor motorA, motorB, motorC;

    public int lastAngle;

    // ================== THÊM BIẾN LED Ở ĐÂY ==================
    RevBlinkinLedDriver LED;

    @Override
    public void runOpMode() throws InterruptedException {
        // 2. Ánh xạ phần cứng (hardwareMap)
        //drive train
        MLeftBackDrive = hardwareMap.get(DcMotor.class, "m6");
        MLeftFrontDrive = hardwareMap.get(DcMotor.class, "m5");
        MRightBackDrive = hardwareMap.get(DcMotor.class, "m2");
        MRightFrontDrive = hardwareMap.get(DcMotor.class, "m1");
        // Can Gat
        SCanGat1 = hardwareMap.get(Servo.class, "s4");
        SCanGat2 = hardwareMap.get(Servo.class, "s5");
        SCanGat3 = hardwareMap.get(Servo.class, "s3");
        // Kep Duoi
        SKepDuoi = hardwareMap.get(Servo.class, "s1");
        // Kep Tren
        SKepTren = hardwareMap.get(Servo.class, "6");  //**** Ví sao chỉ để số 6
        // Can Xoay Tren
        SCanXoayTren1 = hardwareMap.get(Servo.class, "s7");
        SCanXoayTren2 = hardwareMap.get(Servo.class, "s8");
        // Thanh Dung
        MThanhDung1 = hardwareMap.get(DcMotor.class, "m7");
        MThanhDung2 = hardwareMap.get(DcMotor.class, "m8");
        MThanhDung3 = hardwareMap.get(DcMotor.class, "m4");
        // Can Xoay Nho Tren
        SCanXoayNhoTren = hardwareMap.get(Servo.class, "s9");
        // Can Thut Tren
        SCanThutTren = hardwareMap.get(Servo.class, "s10");
        // Can Xoay Ngang Duoi
        SCanXoayNgangDuoi = hardwareMap.get(Servo.class, "s2");
        // Thanh truot duoi
        MThanhTruotDuoi = hardwareMap.get(DcMotor.class, "m3");
        // Sensor
        Dsdistance = hardwareMap.get(DistanceSensor.class, "distance");
        //DsdistanceUp = hardwareMap.get(DistanceSensor.class, "distanceUp");
        DsBackDis = hardwareMap.get(DistanceSensor.class, "backdis");
        ClColor = hardwareMap.get(ColorSensor.class, "color");
        // ================== ÁNH XẠ LED ==================
        LED = hardwareMap.get(RevBlinkinLedDriver.class, "LED5");

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        LED.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);

        // 3. Cài đặt chế độ motor, hướng quay, initial vị trí servo, reset encoder...
        //-------Hướng quay của motor
        MThanhDung3.setDirection(DcMotorSimple.Direction.REVERSE);
        //MThanhDung1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); Xóa đi vì chê độ auto chay trước đó
        MThanhDung1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MThanhTruotDuoi.setDirection(DcMotorSimple.Direction.FORWARD);
        MLeftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        MLeftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        //-------Initial vị trí


        // 4. Chờ người dùng nhấn START trên Driver Station
        waitForStart();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        DualServoCanGatSetPosition(SCanXoayTren1, SCanXoayTren2, 0.6994);
        SCanXoayNhoTren.setPosition(0.4456);
        SCanThutTren.setPosition(0.25);
        SKepTren.setPosition(FKepTrenNoTakeSample);

        // 5. Chạy vòng lặp chính khi robot đang hoạt động
        while (opModeIsActive()) {
            // 6. Đọc input từ gamepad, xử lý logic, điều khiển robot
            //-------This code use for control drivetrain
//            double y = -gamepad1.left_stick_y;
//            double x = gamepad1.left_stick_x;
//            double rx = gamepad1.right_stick_x;
//
//            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//
//            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
//            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
//
//            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
//
//            MLeftBackDrive.setPower((rotY - rotX + rx) / denominator);
//            MLeftFrontDrive.setPower((rotY + rotX + rx) / denominator);
//            MLeftBackDrive.setPower((rotY + rotX - rx) / denominator);
//            MRightFrontDrive.setPower((rotY - rotX - rx) / denominator);
//
//            lastAngle = (int) Math.toDegrees(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

            MRightFrontDrive.setPower(-(((gamepad1.left_stick_y + gamepad1.right_stick_y) + gamepad1.left_stick_x + gamepad1.right_stick_x) + (gamepad2.right_stick_y + gamepad2.left_stick_x + gamepad2.right_stick_x)));
            MLeftFrontDrive.setPower(-(((gamepad1.left_stick_y + gamepad1.right_stick_y) - gamepad1.left_stick_x - gamepad1.right_stick_x) + (gamepad2.right_stick_y - gamepad2.left_stick_x - gamepad2.right_stick_x)));;
            MRightBackDrive.setPower(-(((gamepad1.left_stick_y + gamepad1.right_stick_y) - gamepad1.left_stick_x + gamepad1.right_stick_x) + (gamepad2.right_stick_y - gamepad2.left_stick_x + gamepad2.right_stick_x)));
            MLeftBackDrive.setPower(-(((gamepad1.left_stick_y + gamepad1.right_stick_y) + gamepad1.left_stick_x - gamepad1.right_stick_x) + (gamepad2.right_stick_y + gamepad2.left_stick_x - gamepad2.right_stick_x)));

            // ================== CHUYỂN CHẾ ĐỘ Ở ĐÂY ==================

            //------------Leo----------------
            if (gamepad2.share) {
                DualServoCanGatSetPosition(SCanXoayTren1, SCanXoayTren2, 0.5);
                SCanXoayNhoTren.setPosition(0.4694);
            }

            ///-------- Điều khiển thanh ngang bằng tay ----
            if (Math.abs(gamepad2.left_stick_y) > 0.05) {
                if (gamepad2.left_stick_y < 0 && BatCamera){
                    BCamera = true;  // bật xuay theo camera
                    BAutoTakeSampleYellow = false; // không yêu cần thanh trược xuông lấy màu vàng
                }
                int pos = MThanhTruotDuoi.getCurrentPosition();
                // Chặn nếu vượt quá hành trình cho phép
                if ((pos >= -10 && gamepad2.left_stick_y > 0) || (pos <= -650 && gamepad2.left_stick_y < 0)) {
                    MThanhTruotDuoi.setPower(0);  // Vượt giới hạn, không cho chạy
                } else {
                    MThanhTruotDuoi.setPower(gamepad2.left_stick_y);  // Trong giới hạn, cho chạy
                }
                if (pos < -150) {
                    if (SCanGat1.getPosition() > 0.30){
                        SCanGat3.setPosition(FCanGat3TakeSamplePos);
                    } else {
                        new Thread(() -> {
                            SCanGat3.setPosition(1);
                            sleep(200);  // delay an toàn không làm chậm vòng lặp chính
                            telemetry.addData("pos < -150",pos);
                            DualServoCanGatSetPosition(SCanGat1, SCanGat2, 0.36);
                            SCanGat3.setPosition(FCanGat3TakeSamplePos);
                        }).start();
                    }
                } else if (pos > -100 && gamepad2.left_stick_y > 0) {
                    sleep(150);                                              // Delay chờ servo
                    RunMotorWithPos(MThanhTruotDuoi, -41, 0.5);
                    SCanXoayTren2.setPosition(0.3006);
                    SCanXoayNhoTren.setPosition(0.4856);
                    SCanThutTren.setPosition(0.25);
                    SCanGat3.setPosition(0.5583);
                    DualServoCanGatSetPosition(SCanGat1, SCanGat2, 0.15);
                } else {
                    BAutoTakeSample = false;
                }

            } else {
                MThanhTruotDuoi.setPower(0);
            }

            /// -------- Điểu khiển cần trên------------
            if (currentMode == ColorMode.BLUE_ONLY) {
                if (gamepad1.right_bumper && !isMovingUp) {
                    SKepTren.setPosition(FKepTrenTakeSample);
                    new Thread(() -> {
                        try {
                            Thread.sleep(300);  // Delay servo trước khi motor chạy
                        } catch (InterruptedException e) {}
                        BThanhDungauto = true;
                        isMovingUp = true;  // Thanh trượt đi lên
                    }).start();
                }

                if (gamepad1.left_bumper && !isMovingDown) {
                    new Thread(() -> {
                        SKepTren.setPosition(FKepTrenNoTakeSample);
                        try {
                            Thread.sleep(300);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }

                        DualServoCanGatSetPosition(SCanXoayTren1, SCanXoayTren2, 0);
                        SCanXoayNhoTren.setPosition(0.4694);
                        SCanThutTren.setPosition(0.49833333333333335);
                        BThanhDungauto = true;
                        isMovingDown = true;
                    }).start();
                }
            }

            updateMotorPositionBlue();          // điều khiển thanh trượt đi lên xuống blue

            ///--------This code use to control Can Xoay Ngang
            if (gamepad2.right_trigger > 0.25 && TCanXoayNgangDuoi.milliseconds() > 100) {
                BCamera = false; // Tắt xuay bằng camera
                BatCamera = false;
                BAutoTakeSample = false;
                SCanXoayNgangDuoi.setPosition(SCanXoayNgangDuoi.getPosition() - gamepad2.right_trigger / 100);
                TCanXoayNgangDuoi.reset();
            }
            if (gamepad2.left_trigger > 0.25 && TCanXoayNgangDuoi.milliseconds() > 100) {
                BCamera = false; // Tắt xuay bằng camera
                BatCamera = false;
                BAutoTakeSample = false;
                SCanXoayNgangDuoi.setPosition(SCanXoayNgangDuoi.getPosition() + gamepad2.left_trigger / 100);
                TCanXoayNgangDuoi.reset();
            }

            /// ----- Đóng/mở kẹp intake sample -----
            if (gamepad2.left_bumper) {
                SKepDuoi.setPosition(FKepNoTakeSample);  // Mở kẹp dưới
            }

            if (gamepad2.right_bumper) {

                new Thread(() -> {
                    SKepTren.setPosition(FKepNoTakeSample);                             // Mở kẹp trên
                    DualServoCanGatSetPosition(SCanGat1, SCanGat2, 0.42);       // Cần gắp ra giữa
                    sleep(200);                                              // Delay an toàn
                    SKepDuoi.setPosition(FKepTakeSample);                               // Kẹp dưới đóng lại (gắp)
                    sleep(200);                                              // Delay chờ servo
                    DualServoCanGatSetPosition(SCanGat1, SCanGat2, 0.36);       // Cần gắp thu vào
                }).start();
            }

            ///-------- Điều khiển thanh đứng bằng tay----
            if (gamepad1.right_trigger > 0.1 && MThanhDung2.getCurrentPosition() < 5000) { //**kiểm tra lại số này
                BThanhDungauto = false;
                isMovingUp = false;
                isMovingDown = false;
                double power = gamepad1.right_trigger;

                MThanhDung1.setPower(power);
                MThanhDung2.setPower(power);
                MThanhDung3.setPower(power);

            } else if (gamepad1.left_trigger > 0.1 && MThanhDung2.getCurrentPosition() > -2) {BThanhDungauto = false;
                isMovingUp = false;
                isMovingDown = false;
                double power = -gamepad1.left_trigger;

                MThanhDung1.setPower(power);
                MThanhDung2.setPower(power);
                MThanhDung3.setPower(power);

            } else if (!BThanhDungauto) {
                MThanhDung1.setPower(0);
                MThanhDung2.setPower(0);
                MThanhDung3.setPower(0);
            }

            ///------Hiển thi Trạng thái của robot
            debugRobotState();

            if(gamepad2.a && TCanGat3.milliseconds() > 100) {
                SCanGat3.setPosition(SCanGat3.getPosition() + 0.002);
                TCanGat3.reset();
            } else if(gamepad2.y && TCanGat3.milliseconds() > 100){
                SCanGat3.setPosition(SCanGat3.getPosition()-0.002);
                TCanGat3.reset();
            }

//            if(gamepad1.dpad_up && TCanXoaytren.milliseconds() > 100) {
//                DualServoCanGatSetPosition(SCanXoayTren1, SCanXoayTren2, SCanXoayTren1.getPosition()+0.05);
//                TCanXoaytren.reset();
//            } else if(gamepad1.dpad_down && TCanXoaytren.milliseconds() > 100){
//                DualServoCanGatSetPosition(SCanXoayTren1, SCanXoayTren2, SCanXoayTren1.getPosition()-0.05);
//                TCanXoaytren.reset();
//            }
//            if(gamepad1.dpad_up && TCanXoaytren.milliseconds() > 100) {
//                SCanXoayNhoTren.setPosition(SCanXoayNhoTren.getPosition() + 0.02);
//                TCanXoaytren.reset();
//            } else if(gamepad1.dpad_down && TCanXoaytren.milliseconds() > 100){
//                SCanXoayNhoTren.setPosition(SCanXoayNhoTren.getPosition() - 0.02);
//                TCanXoaytren.reset();
//            }
            if(gamepad1.dpad_up && TCanGat.milliseconds() > 100) {
                DualServoCanGatSetPosition(SCanGat1, SCanGat2, SCanGat1.getPosition()+0.002);
                TCanXoaytren.reset();
            } else if(gamepad1.dpad_down && TCanGat.milliseconds() > 100){
                DualServoCanGatSetPosition(SCanGat1, SCanGat2, SCanGat1.getPosition()-0.002);
                TCanXoaytren.reset();
            }
        }

    }

    // ✅ HÀM NÊN KHAI BÁO Ở ĐÂY – SAU runOpMode(), NHƯNG TRONG CLASS
    /// ---- Điểu khiển motor thanh trượt tự động-----

    public void updateMotorPositionBlue() {
        /// Thanh đứng nâng lên
        if (isMovingUp) {
            int currentPos = MThanhDung1.getCurrentPosition();
            if (currentPos < 1360) {
                BThanhDungauto = true;
                MThanhDung1.setPower(1);
                MThanhDung2.setPower(1);
                MThanhDung3.setPower(1);
            } else {
                MThanhDung1.setPower(0);
                MThanhDung2.setPower(0);
                MThanhDung3.setPower(0);
                isMovingUp = false;
                // Tiếp tục hành động khi lên xong
                DualServoCanGatSetPosition(SCanXoayTren1, SCanXoayTren2, 0.6994);
                SCanXoayNhoTren.setPosition(0.4311);
                SCanThutTren.setPosition(0.7989);
            }
        }

        // Thanh đứng hạ xuống
        if (isMovingDown) {
            int currentPos = MThanhDung1.getCurrentPosition();
            if (currentPos > 20) {
                BThanhDungauto = true;
                MThanhDung1.setPower(-1);
                MThanhDung2.setPower(-1);
                MThanhDung3.setPower(-1);
            } else {
                MThanhDung1.setPower(0);
                MThanhDung2.setPower(0);
                MThanhDung3.setPower(0);
                isMovingDown = false;

                SKepTren.setPosition(FKepTrenNoTakeSample);
            }
        }
    }

    // Hàm phát hiện màu
    public String detectColor(ColorSensor colorSensor) {
        float[] hsv = new float[3];
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();

        Color.RGBToHSV(r * 255 / 800, g * 255 / 800, b * 255 / 800, hsv);
        float hue = hsv[0];
        float sat = hsv[1];
        float val = hsv[2];

        if (hue >= 10 && hue <= 240 && sat > 0.5 && val > 1.5) { //hue >= 210 && hue <= 240 && sat > 0.5 && val > 1.5
            return "Blue";
        }
//        else if (hue >= 65 && hue <= 95 && sat > 0.5 && val > 1.5) {
//            return "Yellow";
//        } else if (hue >= 10 && hue <= 30 && sat > 0.5 && val > 1.5) {
//            return "Red";
//        }
        else {
            return "Unknown";
        }
    }

    // Ham điều khiển 2 servo của cần gắp dưới
    public void DualServoCanGatSetPosition(Servo servo1, Servo servo2, double position) {
        servo1.setPosition(position);
        servo2.setPosition(1 - position);
    }

    // Hàm hien thi các thông só của robot
    public void debugRobotState() {
        // ▶️ Thông tin Servo
        telemetry.addLine("=== Servo Positions ===");
        telemetry.addData("Cần Gạt 1", SCanGat1.getPosition());
        telemetry.addData("Cần Gạt 3", SCanGat3.getPosition());
        telemetry.addData("Cần Xoay Ngang Dưới", SCanXoayNgangDuoi.getPosition());
        telemetry.addData("Cần Thụt Trên", SCanThutTren.getPosition());
        telemetry.addData("Cần Xoay Trên", SCanXoayTren2.getPosition());
        telemetry.addData("Cần Xoay Nhỏ Trên", SCanXoayNhoTren.getPosition());
        telemetry.addData("Kẹp Trên", SKepTren.getPosition());
        telemetry.addData("Kẹp Dưới", SKepDuoi.getPosition());

        // ▶️ Vị trí Encoder của các động cơ
        telemetry.addLine("=== Encoder Values ===");
        telemetry.addData("Thanh Trượt Dưới", MThanhTruotDuoi.getCurrentPosition());
        telemetry.addData("Thanh Đứng", MThanhDung1.getCurrentPosition());
        telemetry.addData("Thanh Đứng 2", MThanhDung2.getCurrentPosition());
        telemetry.addData("Thanh Đứng 3", MThanhDung3.getCurrentPosition());

        // ▶️ Cảm biến khoảng cách
        telemetry.addLine("=== Sensor Data ===");
        telemetry.addData("Khoảng cách (cm)", Dsdistance.getDistance(DistanceUnit.CM));
        telemetry.addData("Position CatGat1", SCanGat1.getPosition());
        telemetry.addData("gamepad2.left_stick_y", gamepad2.left_stick_y);


        // ▶️ Xử lý ảnh từ Camera
//        telemetry.addLine("=== Vision / Sample Data ===");
//        telemetry.addData("Y của vật thể", samplePipeline.CAMERA_HEIGHT - samplePipeline.cY);
//        telemetry.addData("Góc điều chỉnh", ((samplePipeline.sampleAngle - 0) / 1800) + 0.2733);
//        telemetry.addData("Góc thô", samplePipeline.sampleAngle);
//        telemetry.addData("Tự động camera", BAutoTakeSample);

        telemetry.update();
    }

    // Hiển thị cảm biến màu
    public void telemetryColorSensor(ColorSensor colorSensor) {
        float[] hsv = new float[3];

        // Đọc giá trị RGB từ cảm biến
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();

        // Chuyển RGB → HSV
        Color.RGBToHSV(r * 255 / 800, g * 255 / 800, b * 255 / 800, hsv);
        float hue = hsv[0];
        float sat = hsv[1];
        float val = hsv[2];

        // Xác định tên màu
        String detectedColor;
        if (hue >= 210 && hue <= 240 && sat > 0.5 && val > 1.5) { //hue >= 10 && hue <= 240 && sat > 0.5 && val > 1.5
            detectedColor = "Red";
        } else if (hue >= 65 && hue <= 95 && sat > 0.5 && val > 1.5) {
            detectedColor = "Yellow";
        } else if (hue >= 10 && hue <= 240 && sat > 0.5 && val > 1.5) { //hue >= 10 && hue <= 240 && sat > 0.5 && val > 1.5
            detectedColor = "Blue";
        } else {
            detectedColor = "Unknown";
        }

        // Hiển thị telemetry
        telemetry.addLine("=== Color Sensor ===");
        telemetry.addData("Raw RGB", "R: %d  G: %d  B: %d", r, g, b);
        telemetry.addData("HSV", "Hue: %.1f  Sat: %.2f  Val: %.2f", hue, sat, val);
        telemetry.addData("Detected Color", detectedColor);
        telemetry.addData("gamepad2.stick", detectedColor);
        telemetry.update();
    }

    public void RunMotorWithPos(DcMotor motor, double pos, double power){ /// Use to run DcMotor with encoder
        motor.setTargetPosition((int) pos);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(motor.getCurrentPosition() < pos){
            motor.setPower(power);
        } else {
            motor.setPower(-power);
        }
//        motor.setPower(power);
        while (motor.isBusy()){
            debugRobotState();
        }
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}