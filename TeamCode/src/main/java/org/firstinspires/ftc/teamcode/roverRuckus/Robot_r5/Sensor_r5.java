package org.firstinspires.ftc.teamcode.roverRuckus.Robot_r5;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "r5.sensor", group = "r4")

public class Sensor_r5 extends Config_r5 {
    @Override
    public void init() {
        config(this);
    }

    @Override
    public void loop() {
        telemetry.addData("dt0", drive.rightFront.getEncoderPosition());
        telemetry.addData("dt1", drive.leftFront.getEncoderPosition());
        telemetry.addData("dt2", drive.leftBack.getEncoderPosition());
        telemetry.addData("dt3", drive.rightBack.getEncoderPosition());

        telemetry.addData("lift", lift.extension.getEncoderPosition());

        telemetry.addData("gyro1", gyro.getOrientation().firstAngle);
        telemetry.addData("gyro2", gyro.getOrientation().secondAngle);
        telemetry.addData("gyro3", gyro.getOrientation().thirdAngle);


        telemetry.update();
    }
}
