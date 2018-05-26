package org.firstinspires.ftc.teamcode.PineappleRobotPackage.Examples.DriveExample;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleEnum;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleMotor;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleRobot;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.Sensors.PineappleOpticalDistanceSensor;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.Sensors.PineappleTouchSensor;

/**
 * Created by Brandon on 8/14/2017.
 */
@TeleOp(name = "PineEx-LineFollow", group = "Linear Opmode")
@Disabled

public class LineFollowExample extends LinearOpMode{

    PineappleRobot robot;

    PineappleMotor left;
    PineappleMotor right;

    PineappleOpticalDistanceSensor color;
    PineappleTouchSensor touch;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new PineappleRobot(this);

        left = robot.motorHandler.newDriveMotor("left", 2, false, false, PineappleEnum.MotorLoc.LEFT, PineappleEnum.MotorType.NEV40);
        right = robot.motorHandler.newDriveMotor("right", 2, false, false, PineappleEnum.MotorLoc.RIGHT, PineappleEnum.MotorType.NEV40);

        color = robot.sensorHandler.newOpticalDistanceSensor("o");
        touch = robot.sensorHandler.newTouchSensor("t");

        right.motorObject.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        robot.auto.lineFollow(color, PineappleEnum.PineappleSensorEnum.ODSLIGHTDETECTED,touch, PineappleEnum.PineappleSensorEnum.TOUCH, PineappleEnum.Condition.GREATERTHAN, 90, -.4);

    }
}
