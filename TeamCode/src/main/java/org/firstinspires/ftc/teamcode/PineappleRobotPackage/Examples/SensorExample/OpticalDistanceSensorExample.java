package org.firstinspires.ftc.teamcode.PineappleRobotPackage.Examples.SensorExample;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleEnum;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleRobot;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.Sensors.PineappleOpticalDistanceSensor;

/**
 * Created by young on 8/7/2017.
 */
@TeleOp(name = "PineEx-OpticalDistance", group = "Linear Opmode")
@Disabled


public class OpticalDistanceSensorExample extends LinearOpMode {

    PineappleRobot robot;

    PineappleOpticalDistanceSensor optical;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new PineappleRobot(this);

        optical = robot.sensorHandler.newOpticalDistanceSensor("o");

        waitForStart();
        optical.ODSLEDToggle(true);

        while(opModeIsActive()) {
            robot.sayFeedBack("Raw", optical.getValue(PineappleEnum.PineappleSensorEnum.ODSRAW));
            robot.sayFeedBack("LightDetected", optical.getValue(PineappleEnum.PineappleSensorEnum.ODSLIGHTDETECTED));
            robot.sayFeedBack("Raw Max", optical.getValue(PineappleEnum.PineappleSensorEnum.ODSRAWMAX));
            robot.updateFeedBack();
        }
    }
}