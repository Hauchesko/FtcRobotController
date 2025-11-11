package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class VariablePractice extends OpMode {
    @Override
    public void init() {
        int TeamNumber = 23014;
        double MotorSpeed = 0.75;
        boolean ClawClosed = true;
        String TeamName = "MMM";
        int MotorAngle = 90;

        telemetry.addData("Team Number", TeamNumber);
        telemetry.addData("Motor Speed", MotorSpeed);
        telemetry.addData("Claw Closed", ClawClosed);
        telemetry.addData("Team Name", TeamName);
        telemetry.addData("Motor Angle", MotorAngle);
    }

    @Override
    public void loop() {

    }
}
