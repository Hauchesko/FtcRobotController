package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class IfPractice extends OpMode {

    @Override
    public void init() {

    }

    @Override
    public void loop() {


        boolean aButton = gamepad1.a;
        if (aButton) {
            telemetry.addData("A button", "Pressed!");
        }
        else {
            telemetry.addData("A button", "Not Pressed!");
        }
        telemetry.addData("A button State", aButton);



        double leftY = gamepad1.left_stick_y;

        if (leftY < 0.1 && leftY > -0.1) {
            telemetry.addData("Left Stick", "In dead zone");
        }

        if (leftY < 0) {
            telemetry.addData("Left Stick", "Is negative");
        }
        else if (leftY > 0.5) {
            telemetry.addData("Left Stick", "Greater than 50%");
        }
        else if (leftY > 0) {
            telemetry.addData("Left Stick", "Is Greater than 0");
        }
        else {
            telemetry.addData("Left Stick", "Is Zero");
        }
        telemetry.addData("Left Stick value", leftY);


        if (!gamepad1.a) {
            leftY *= 0.5;
        }
        telemetry.addData("Speed", leftY);

    }
}
