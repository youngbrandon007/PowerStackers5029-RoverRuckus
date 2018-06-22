package org.firstinspires.ftc.teamcode.PineappleRobotPackage.Examples.SensorExample;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PSEnum;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PSRobot;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.Sensors.PSUltrasonicSensor;

/**
 * Created by young on 8/7/2017.
 */

@TeleOp(name = "PineEx-UltrasonicSensor", group = "Linear Opmode")
@Disabled


public class UltrasonicSensorExample extends LinearOpMode {

    PSRobot robot;

    PSUltrasonicSensor ultrasonicSensor;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new PSRobot(this);

        ultrasonicSensor = robot.sensorHandler.newUltrasonicSensor("us");

        waitForStart();
        while(opModeIsActive()) {
            robot.sayFeedBack(ultrasonicSensor.sensorName, ultrasonicSensor.getValue(PSEnum.PineappleSensorEnum.US));
            robot.updateFeedBack();
        }
    }
}
