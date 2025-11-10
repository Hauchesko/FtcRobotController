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

        telemetry.addData("Team Number", TeamNumber);
    }

    @Override
    public void loop() {

    }
}
