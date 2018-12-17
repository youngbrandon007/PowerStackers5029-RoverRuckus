package org.firstinspires.ftc.teamcode.roverRuckus.Robot_r3;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "r3.sensor", group = "r3")
public class Sensor_r3 extends Config_r3 {
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

        telemetry.addData("gyro", gyro.getAngle1());
        telemetry.addData("gyro", gyro.getAngle2());
        telemetry.addData("gyro", gyro.getAngle3());

        telemetry.update();
    }
}
