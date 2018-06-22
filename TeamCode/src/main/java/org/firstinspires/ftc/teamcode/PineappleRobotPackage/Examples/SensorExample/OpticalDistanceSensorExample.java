package org.firstinspires.ftc.teamcode.PineappleRobotPackage.Examples.SensorExample;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PSEnum;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PSRobot;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.Sensors.PSOpticalDistanceSensor;

/**
 * Created by young on 8/7/2017.
 */
@TeleOp(name = "PineEx-OpticalDistance", group = "Linear Opmode")
@Disabled


public class OpticalDistanceSensorExample extends LinearOpMode {

    PSRobot robot;

    PSOpticalDistanceSensor optical;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new PSRobot(this);

        optical = robot.sensorHandler.newOpticalDistanceSensor("o");

        waitForStart();
        optical.ODSLEDToggle(true);

        while(opModeIsActive()) {
            robot.sayFeedBack("Raw", optical.getValue(PSEnum.PineappleSensorEnum.ODSRAW));
            robot.sayFeedBack("LightDetected", optical.getValue(PSEnum.PineappleSensorEnum.ODSLIGHTDETECTED));
            robot.sayFeedBack("Raw Max", optical.getValue(PSEnum.PineappleSensorEnum.ODSRAWMAX));
            robot.updateFeedBack();
        }
    }
}