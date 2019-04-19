package org.firstinspires.ftc.teamcode.roverRuckus.Robot_r5;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "r5.Config",group = "r5")
public class TestConfig_r5 extends OpMode{

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
