package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Simple Two Motor Drive", group = "FTC Advisor")
public class SimpleTwoMotorTeleOp extends LinearOpMode {

    // Объявляем моторы
    private DcMotor leftMotor;
    private DcMotor rightMotor;

    @Override
    public void runOpMode() {

        // Подключаем моторы по названиям из конфигурации (Driver Hub)
        leftMotor  = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");

        // Устанавливаем направление вращения (чтобы оба ехали вперёд одинаково)
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Сбрасываем энкодеры и ставим режим обычного управления
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Ready to Start");
        telemetry.update();

        // Ждём нажатия Play на пульте
        waitForStart();

        while (opModeIsActive()) {

            // Управление стиками: левый - движение, правый - поворот
            double drive  = -gamepad1.left_stick_y;   // Вперёд/назад
            double turn   = gamepad1.right_stick_x;   // Поворот

            // Рассчитываем мощность для каждого мотора
            double leftPower  = drive + turn;
            double rightPower = drive - turn;

            // Ограничиваем значения до диапазона [-1, 1]
            leftPower  = Math.max(-1.0, Math.min(1.0, leftPower));
            rightPower = Math.max(-1.0, Math.min(1.0, rightPower));

            // Устанавливаем мощность моторам
            leftMotor.setPower(leftPower * 0.8);   // 80% максимальной мощности
            rightMotor.setPower(rightPower * 0.8);

            // Отображаем значения в Driver Station
            telemetry.addData("Left Power",  leftPower);
            telemetry.addData("Right Power", rightPower);
            telemetry.update();
        }
    }
}