package org.firstinspires.ftc.teamcode.roverRuckus.Robot_r4;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "r4.sensor", group = "r4")
@Disabled
public class Sensor_r4 extends Config_r4 {
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
