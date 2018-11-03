package org.firstinspires.ftc.teamcode.roverRuckus.Robot_r2;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "r2.encoder", group = "r2")
public class EncoderTesting extends Config{


    @Override
    public void init() {
        config(this);
    }

    @Override
    public void loop() {
//        telemetry.addData("drive.LF", drive.leftFront.getEncoderPosition());
//        telemetry.addData("drive.LB", lb);
//        telemetry.addData("drive.RF", rf);
//        telemetry.addData("drive.RB", rb);
        drive.autoDive(90, 500, .2, 0.0);
    }
}
