package org.firstinspires.ftc.teamcode.PineappleRobotPackage.Examples.DriveExample;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleEnum;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleMotor;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleRobot;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.Sensors.PineappleTouchSensor;

/**
 * Created by young on 8/7/2017.
 */
@TeleOp(name = "PineEx-DriveUntil", group = "Linear Opmode")
@Disabled

public class DriveUntilExample extends LinearOpMode {
    PineappleRobot robot;

    PineappleMotor left;
    PineappleMotor right;
    PineappleTouchSensor touch;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new PineappleRobot(this);

        left = robot.motorHandler.newDriveMotor("left", 1, true , true, PineappleEnum.MotorLoc.LEFT, PineappleEnum.MotorType.NEV40);
        right = robot.motorHandler.newDriveMotor("right", 1, true , true, PineappleEnum.MotorLoc.RIGHT, PineappleEnum.MotorType.NEV40);

        touch = robot.sensorHandler.newTouchSensor("touch");

        right.motorObject.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        robot.auto.driveUntil(touch, PineappleEnum.PineappleSensorEnum.TOUCH, PineappleEnum.Condition.EQUAL, 1, .2);
    }
}
