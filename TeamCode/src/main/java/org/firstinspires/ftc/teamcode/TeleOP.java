package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOP with all Functions", group = "Linear OpMode")
public class TeleOP extends LinearOpMode {

    // МОТОРЫ
    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private DcMotorEx shooter1, shooter2;
    private DcMotor intake;
    private Servo servoDegree;
    private Servo servoGate;

    // ===== КОНСТАНТЫ =====
    private static final double MOTOR_POWER = 0.75;
    double F = 0;
    double P = 0;




    // ===== Серво =====
    double maxServoGatePosition = 1.0;
    double minServoGatePosition = 0;

    double maxServoDegreePosition = 0.8;
    double minServoDegreePosition = 0;
    double currentServoGatePosition = 0.5;
    double currentServoDegreePosition = 0;


    boolean lastDpadUp = false;
    boolean lastDpadDown = false;
    boolean lastDpadRight = false;
    boolean lastDpadLeft = false;

    @Override
    public void runOpMode() {

        // ===== HardwareMap =====
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        intake   = hardwareMap.get(DcMotor.class, "intake");

        servoDegree = hardwareMap.get(Servo.class, "servoDegree");
        servoGate = hardwareMap.get(Servo.class, "servoGate");


        // ===== PIDF for Shooter =====

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);



        // ===== НАПРАВЛЕНИЯ =====
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        servoDegree.setPosition(currentServoDegreePosition);
        servoGate.setPosition(currentServoGatePosition);

        double intakePower = 0;

        waitForStart();

        while (opModeIsActive()) {


            // 1. MECANUM (0.8 MAX)

            double y  = gamepad1.left_stick_y;
            double x  = -gamepad1.left_stick_x * 1.1;
            double rx = -gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            double lf = (y + x + rx) / denominator * MOTOR_POWER;
            double lb = (y - x + rx) / denominator * MOTOR_POWER;
            double rf = (y - x - rx) / denominator * MOTOR_POWER;
            double rb = (y + x - rx) / denominator * MOTOR_POWER;

            leftFront.setPower(lf);
            leftBack.setPower(lb);
            rightFront.setPower(rf);
            rightBack.setPower(rb);



            // 2. SHOOTER (0.8 / -0.8)

            if (gamepad1.rightBumperWasPressed()) {
                F = 22.5;
                P = 35;
            }
            if (gamepad1.rightBumperWasReleased()) {
                P = 0;
                F = 0;
            }

            PIDFCoefficients pidfCoefficients1 = new PIDFCoefficients(P, 0, 0, F);
            shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients1);
            shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients1);

            shooter1.setVelocity(1500);
            shooter2.setVelocity(1500);



            // 3. INTAKE (0.8 / -0.8)

            if (gamepad1.right_trigger>0.1) {
                intakePower = -1;
            }
            else if (gamepad1.left_trigger>0.1) {
                intakePower = 1;
            } else{
                intakePower = 0;
            }

            intake.setPower(intakePower);



            // 4. SERVO DEGREE (D-PAD)

            if (gamepad1.dpad_up && !lastDpadUp) {
                currentServoDegreePosition += 0.05;
            }
            lastDpadUp = gamepad1.dpad_up;

            if (gamepad1.dpad_down && !lastDpadDown) {
                currentServoDegreePosition -= 0.05;
            }
            lastDpadDown = gamepad1.dpad_down;

            if (currentServoDegreePosition > maxServoDegreePosition) {
                currentServoDegreePosition = maxServoDegreePosition;
            } else if (currentServoDegreePosition < minServoDegreePosition) {
                currentServoDegreePosition = minServoDegreePosition;
            }

            servoDegree.setPosition(currentServoDegreePosition);


            // 5. SERVO GATE

            if (gamepad1.dpad_right && !lastDpadRight) {
                currentServoGatePosition += 0.05;
            }
            lastDpadRight = gamepad1.dpad_right;

            if (gamepad1.dpad_left && !lastDpadLeft) {
                currentServoGatePosition -= 0.05;
            }
            lastDpadLeft = gamepad1.dpad_left;

            if (currentServoGatePosition > maxServoGatePosition) {
                currentServoGatePosition = maxServoGatePosition;
            } else if (currentServoGatePosition < minServoGatePosition) {
                currentServoGatePosition = minServoGatePosition;
            }

            servoGate.setPosition(currentServoGatePosition);



            // 6. TELEMETRY

            telemetry.addData("Drive Power", "0.8 MAX");
            telemetry.addData("Intake", intakePower);
            telemetry.addData("Servo Degree", currentServoDegreePosition);
            telemetry.addData("Servo Gate", currentServoGatePosition);
            telemetry.update();
        }
    }
}