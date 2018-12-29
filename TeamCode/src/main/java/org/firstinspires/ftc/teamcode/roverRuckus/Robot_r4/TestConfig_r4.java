package org.firstinspires.ftc.teamcode.roverRuckus.Robot_r4;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "r4.Config",group = "r4")
public class TestConfig_r4 extends OpMode{

    @Override
    public void init() {

    }

    @Override
    public void start() {
        telemetry.addData("CONFIG", LoadConfig.getConfig());
    }

    @Override
    public void loop() {

    }
}
