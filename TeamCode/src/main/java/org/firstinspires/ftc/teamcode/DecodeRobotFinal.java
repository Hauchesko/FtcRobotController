package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name="DECODE: FINAL PRO CODE", group="Main")
public class DecodeRobotFinal extends LinearOpMode {

    // --- ОБОРУДОВАНИЕ ---
    private DcMotorEx leftFront, rightFront, leftBack, rightBack;
    private DcMotor intakeMotor, shooterMotor;
    private Servo tiltServo, feederServo;
    private CRServo turretServo;

    // --- ЗРЕНИЕ И ВЫБОР АЛЬЯНСА ---
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // ПЕРЕМЕННАЯ ВМЕСТО КОНСТАНТЫ: меняется по нажатию кнопки
    private int targetTagId = 24; // По умолчанию Красный (24)
    private String allianceColor = "RED";

    // --- ПАРАМЕТРЫ ДВИЖЕНИЯ ---
    private final double TICKS_PER_INCH = 45.0;
    private final double RAMP_STEP = 0.05;
    private double pLF, pRF, pLB, pRB;

    private boolean autoAimActive = false;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        initHardware();
        initVision();

        // --- ЦИКЛ ВЫБОРА АЛЬЯНСА (ПЕРЕД СТАРТОМ) ---
        while (!isStarted() &&!isStopRequested()) {
            if (gamepad1.x) { // Синий альянс
                targetTagId = 20;
                allianceColor = "BLUE";
            } else if (gamepad1.b) { // Красный альянс
                targetTagId = 24;
                allianceColor = "RED";
            }

            telemetry.addData(">> ВЫБОР АЛЬЯНСА", allianceColor);
            telemetry.addData(">> TARGET TAG ID", targetTagId);
            telemetry.addData("Инструкция", "Нажми 'X' для BLUE, 'B' для RED");
            telemetry.addData("Статус", "Готов к запуску");
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        if (this.getClass().isAnnotationPresent(TeleOp.class)) {
            runTeleOp();
        } else {
            runAutonomous();
        }
    }

    private void runTeleOp() {
        while (opModeIsActive()) {
            // 1. Движение с плавным разгоном
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            pLF = applyRamping(y + x + rx, pLF);
            pRF = applyRamping(y - x - rx, pRF);
            pLB = applyRamping(y - x + rx, pLB);
            pRB = applyRamping(y + x - rx, pRB);

            leftFront.setPower(pLF);
            rightFront.setPower(pRF);
            leftBack.setPower(pLB);
            rightBack.setPower(pRB);

            // 2. Auto-Aim (Кнопка A) - использует выбранный targetTagId
            if (gamepad1.a) {
                autoAimActive =!autoAimActive;
                sleep(250);
            }

            if (autoAimActive) {
                performAutoAim();
            } else {
                turretServo.setPower(0);
            }

            // 3. Интейк (Кнопка B)
            if (gamepad1.b &&!autoAimActive) intakeMotor.setPower(1.0);
            else intakeMotor.setPower(0);

            // 4. Стрельба (Триггер)
            if (gamepad1.right_trigger > 0.1) {
                shooterMotor.setPower(0.95);
                if (gamepad1.right_trigger > 0.8) feederServo.setPosition(1.0);
                else feederServo.setPosition(0.4);
            } else {
                shooterMotor.setPower(0);
                feederServo.setPosition(0.4);
            }

            updateTelemetry();
        }
    }

    public void performAutoAim() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        AprilTagDetection target = null;

        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == targetTagId) { // Используем выбранный ID
                target = detection;
                break;
            }
        }

        if (target!= null) {
            double bearing = target.ftcPose.bearing;
            double turretPower = bearing * 0.035;
            turretServo.setPower(Range.clip(turretPower, -0.6, 0.6));

            double distance = target.ftcPose.range;
            double tiltPosition = 0.3 + (distance - 12) * (0.5 / 36);
            tiltServo.setPosition(Range.clip(tiltPosition, 0.2, 0.9));
        } else {
            turretServo.setPower(0);
        }
    }

    private void runAutonomous() {
        // Логика аналогична предыдущей, но Auto-Aim
        // теперь автоматически наводится на выбранный альянс
        driveByInches(-20, 0.5);
        shootDuringAuto(3);
        //... продолжение цикла сбора
    }

    private void initHardware() {
        leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack   = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotorEx.class, "rightBack");
        intakeMotor  = hardwareMap.get(DcMotor.class, "intakeMotor");
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        tiltServo   = hardwareMap.get(Servo.class, "tiltServo");
        turretServo = hardwareMap.get(CRServo.class, "turretServo");
        feederServo = hardwareMap.get(Servo.class, "feederServo");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void initVision() {
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    private double applyRamping(double target, double current) {
        if (Math.abs(target - current) < RAMP_STEP) return target;
        return current + (target > current? RAMP_STEP : -RAMP_STEP);
    }

    private void shootDuringAuto(int count) {
        shooterMotor.setPower(0.95);
        sleep(1500);
        for (int i = 0; i < count; i++) {
            performAutoAim();
            feederServo.setPosition(1.0);
            sleep(500);
            feederServo.setPosition(0.4);
            sleep(600);
        }
        shooterMotor.setPower(0);
    }

    public void driveByInches(double inches, double power) {
        int moveTicks = (int)(inches * TICKS_PER_INCH);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setTargetPosition(moveTicks);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);
        while (opModeIsActive() && leftFront.isBusy()) { idle(); }
        leftFront.setPower(0);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void updateTelemetry() {
        telemetry.addData("ALLIANCE", allianceColor);
        telemetry.addData("AutoAim Active", autoAimActive);
        telemetry.addData("Target Tag ID", targetTagId);
        telemetry.update();
    }
}