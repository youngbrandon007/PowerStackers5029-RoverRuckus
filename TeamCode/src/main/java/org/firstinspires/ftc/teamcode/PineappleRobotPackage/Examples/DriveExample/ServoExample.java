package org.firstinspires.ftc.teamcode.PineappleRobotPackage.Examples.DriveExample;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleEnum;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleMotor;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleRobot;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleServo;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.Sensors.PineappleTouchSensor;

/**
 * Created by young on 8/7/2017.
 */
@TeleOp(name = "PineEx-ServoCS", group = "Linear Opmode")
@Disabled

public class ServoExample extends LinearOpMode {
    PineappleRobot robot;

    PineappleServo servo;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new PineappleRobot(this);
        servo = robot.servoHandler.newContinuousServo("D", 0.5);

        waitForStart();
            while(opModeIsActive()){
                if (gamepad1.a){
                    servo.setPosition(1);
                } else {
                    servo.setPosition(0.5);
                }
            }
//        robot.drive.encoderDrive(0.5, "4in",4);
//        robot.drive.setDirectPower(-1, -1);
//        Thread.sleep(1000);
    }
}
