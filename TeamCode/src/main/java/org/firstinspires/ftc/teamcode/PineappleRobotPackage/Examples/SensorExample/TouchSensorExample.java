package org.firstinspires.ftc.teamcode.PineappleRobotPackage.Examples.SensorExample;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PSEnum;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PSRobot;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.Sensors.PSTouchSensor;

/**
 * Created by young on 8/7/2017.
 */

@TeleOp(name = "PineEx-TouchSensor", group = "Linear Opmode")
@Disabled


public class TouchSensorExample extends LinearOpMode {

    PSRobot robot;

    PSTouchSensor touch;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new PSRobot(this);

        touch = robot.sensorHandler.newTouchSensor("t");

        waitForStart();
        while(opModeIsActive()) {
            robot.sayFeedBack(touch.sensorName, touch.getValue(PSEnum.PineappleSensorEnum.TOUCH));
            robot.updateFeedBack();
        }
    }
}
