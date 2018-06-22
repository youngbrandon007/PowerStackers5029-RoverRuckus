package org.firstinspires.ftc.teamcode.PineappleRobotPackage.Examples.SensorExample;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PSEnum;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PSRobot;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.Sensors.PSColorSensor;

/**
 * Created by young on 8/7/2017.
 */
@TeleOp(name = "PineEx-ColorSensor", group = "Linear Opmode")
@Disabled

public class ColorSensorExample  extends LinearOpMode {

    PSRobot robot;

    PSColorSensor color;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new PSRobot(this);

        color = robot.sensorHandler.newColorSensor("c");

        waitForStart();
        while(opModeIsActive()) {
            robot.sayFeedBack("alpha", color.getValue(PSEnum.PineappleSensorEnum.CSALPHA));
            robot.sayFeedBack("red", color.getValue(PSEnum.PineappleSensorEnum.CSRED));
            robot.sayFeedBack("green", color.getValue(PSEnum.PineappleSensorEnum.CSBLUE));
            robot.sayFeedBack("blue", color.getValue(PSEnum.PineappleSensorEnum.CSGREEN));
            robot.sayFeedBack("argb", color.getValue(PSEnum.PineappleSensorEnum.CSARGB));
            robot.updateFeedBack();
        }
    }
}