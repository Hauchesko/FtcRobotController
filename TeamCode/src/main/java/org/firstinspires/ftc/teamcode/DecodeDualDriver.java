package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name="DECODE: DUAL DRIVER PRO", group="Main")
public class DecodeDualDriver extends LinearOpMode {

    // --- ОБОРУДОВАНИЕ ---
    private DcMotorEx leftFront, rightFront, leftBack, rightBack;
    private DcMotor intakeMotor, shooterMotor;
    private Servo tiltServo, feederServo;
    private CRServo turretServo;

    // --- ЗРЕНИЕ И АЛЬЯНС ---
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private int targetTagId = 24;
    private String allianceColor = "RED";

    // --- ПАРАМЕТРЫ ДВИЖЕНИЯ ---
    private final double RAMP_STEP = 0.08;
    private double pLF, pRF, pLB, pRB;

    private boolean autoAimEnabled = false;
    private boolean lastButtonA2 = false;

    @Override
    public void runOpMode() {
        initHardware();
        initVision();

        // --- ВЫБОР АЛЬЯНСА (Геймпад 1) ---
        while (!isStarted() &&!isStopRequested()) {
            if (gamepad1.x) { targetTagId = 20; allianceColor = "BLUE"; }
            if (gamepad1.b) { targetTagId = 24; allianceColor = "RED"; }
            telemetry.addData(">> ВЫБОР АЛЬЯНСА", allianceColor);
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            // ==========================================
            // ДРАЙВЕР 1: ДВИЖЕНИЕ И ИНТЕЙК
            // ==========================================

            // 1. Mecanum Drive (Левый стик - движение, правый - поворот)
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            pLF = applyRamping(y + x + rx, pLF);
            pRF = applyRamping(y - x - rx, pRF);
            pLB = applyRamping(y - x + rx, pLB);
            pRB = applyRamping(y + x - rx, pRB);

            leftFront.setPower(pLF);
            rightFront.setPower(pRF);
            leftBack.setPower(pLB);
            rightBack.setPower(pRB);

            // 2. Интейк (Правый триггер Геймпада 1)
            if (gamepad1.right_trigger > 0.1) intakeMotor.setPower(1.0);
            else if (gamepad1.left_trigger > 0.1) intakeMotor.setPower(-0.8); // Реверс
            else intakeMotor.setPower(0);

            // ==========================================
            // ДРАЙВЕР 2: ШУТЕР, ФИДЕР И НАВЕДЕНИЕ
            // ==========================================

            // 1. Auto-Aim Toggle (Кнопка A на Геймпаде 2)
            if (gamepad2.a &&!lastButtonA2) {
                autoAimEnabled =!autoAimEnabled;
            }
            lastButtonA2 = gamepad2.a;

            if (autoAimEnabled) {
                performAutoAim();
            } else {
                // Ручное управление турелью (левый стик Геймпада 2)
                turretServo.setPower(-gamepad2.left_stick_x * 0.5);
                // Ручное управление углом шутера (правый стик Геймпада 2)
                if (Math.abs(gamepad2.right_stick_y) > 0.1) {
                    double currentPos = tiltServo.getPosition();
                    tiltServo.setPosition(Range.clip(currentPos - gamepad2.right_stick_y * 0.01, 0.2, 0.9));
                }
            }

            // 2. Шутер (Правый триггер Геймпада 2)
            if (gamepad2.right_trigger > 0.1) {
                shooterMotor.setPower(0.95);
            } else {
                shooterMotor.setPower(0);
            }

            // 3. Подача (Фидер) - Кнопка B на Геймпаде 2
            if (gamepad2.b) feederServo.setPosition(1.0); // Толкаем в шутер
            else feederServo.setPosition(0.4);            // Возврат

            updateTelemetry();
        }
    }

    public void performAutoAim() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        AprilTagDetection target = null;
        for (AprilTagDetection d : detections) {
            if (d.id == targetTagId) { target = d; break; }
        }

        if (target!= null) {
            // Горизонталь
            double turretPower = target.ftcPose.bearing * 0.04;
            turretServo.setPower(Range.clip(turretPower, -0.6, 0.6));
            // Вертикаль
            double tiltPos = 0.3 + (target.ftcPose.range - 12) * (0.5 / 40);
            tiltServo.setPosition(Range.clip(tiltPos, 0.2, 0.9));
        } else {
            turretServo.setPower(0);
        }
    }

    private void initHardware() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        tiltServo = hardwareMap.get(Servo.class, "tiltServo");
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

    private void updateTelemetry() {
        telemetry.addData("DRIVER 2", "Aim: " + (autoAimEnabled? "AUTO" : "MANUAL"));
        telemetry.addData("ALLIANCE", allianceColor);
        telemetry.addData("SHOOTER SPEED", "%.2f", shooterMotor.getPower());
        telemetry.update();
    }
}