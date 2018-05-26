package org.firstinspires.ftc.teamcode.PineappleRobotPackage.Examples.SensorExample;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleEnum;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleRobot;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.Sensors.PineappleColorSensor;

/**
 * Created by young on 8/7/2017.
 */
@TeleOp(name = "PineEx-ColorSensor", group = "Linear Opmode")
@Disabled

public class ColorSensorExample  extends LinearOpMode {

    PineappleRobot robot;

    PineappleColorSensor color;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new PineappleRobot(this);

        color = robot.sensorHandler.newColorSensor("c");

        waitForStart();
        while(opModeIsActive()) {
            robot.sayFeedBack("alpha", color.getValue(PineappleEnum.PineappleSensorEnum.CSALPHA));
            robot.sayFeedBack("red", color.getValue(PineappleEnum.PineappleSensorEnum.CSRED));
            robot.sayFeedBack("green", color.getValue(PineappleEnum.PineappleSensorEnum.CSBLUE));
            robot.sayFeedBack("blue", color.getValue(PineappleEnum.PineappleSensorEnum.CSGREEN));
            robot.sayFeedBack("argb", color.getValue(PineappleEnum.PineappleSensorEnum.CSARGB));
            robot.updateFeedBack();
        }
    }
}