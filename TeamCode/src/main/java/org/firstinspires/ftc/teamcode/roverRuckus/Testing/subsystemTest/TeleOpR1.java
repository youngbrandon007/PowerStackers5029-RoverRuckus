package org.firstinspires.ftc.teamcode.roverRuckus.Testing.subsystemTest;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

@TeleOp(name = "Subsystem", group = "test")
@Disabled
public class TeleOpR1 extends config{
    @Override
    public void init() {
        config(this);
    }

    @Override
    public void loop() {
        testMotor();
        telemetry.addData("GAMEPAD.X",gamepad1.left_stick_x);
        telemetry.addData("GAMEPAD.Y",gamepad1.left_stick_y);
    }
}
