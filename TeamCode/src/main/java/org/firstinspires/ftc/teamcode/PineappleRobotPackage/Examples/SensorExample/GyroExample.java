package org.firstinspires.ftc.teamcode.PineappleRobotPackage.Examples.SensorExample;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PSEnum;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PSRobot;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.Sensors.PSGyroSensor;

/**
 * Created by young on 8/7/2017.
 */
@TeleOp(name = "PineEx-GyroSensor", group = "Linear Opmode")
@Disabled


public class GyroExample  extends LinearOpMode {

    PSRobot robot;

    PSGyroSensor gyro;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new PSRobot(this);

        gyro = robot.sensorHandler.newGyroSensor("g");

        waitForStart();

        while(opModeIsActive()) {
            robot.sayFeedBack("Heading", gyro.getValue(PSEnum.PineappleSensorEnum.GSHEADING));
            robot.sayFeedBack("Status", gyro.GSStatus());

            robot.sayFeedBack("X", gyro.getValue(PSEnum.PineappleSensorEnum.GSX));
            robot.sayFeedBack("Y", gyro.getValue(PSEnum.PineappleSensorEnum.GSY));
            robot.sayFeedBack("Z", gyro.getValue(PSEnum.PineappleSensorEnum.GSZ));
            robot.updateFeedBack();
        }
    }
}
