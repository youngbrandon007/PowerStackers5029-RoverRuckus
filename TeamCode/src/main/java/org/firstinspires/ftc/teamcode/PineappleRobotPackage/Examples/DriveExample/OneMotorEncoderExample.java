package org.firstinspires.ftc.teamcode.PineappleRobotPackage.Examples.DriveExample;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleEnum;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleMotor;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleRobot;

/**
 * Created by young on 8/2/2017.
 */


@TeleOp(name = "PineEx-EncoderOneMotor", group = "Linear Opmode")
@Disabled

public class OneMotorEncoderExample extends LinearOpMode {
    PineappleRobot robot;

    PineappleMotor motor;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new PineappleRobot(this);

        motor = robot.motorHandler.newMotor("motor", 1, true , true, PineappleEnum.MotorType.NEV40);

        waitForStart();
        //motor.encoderDrive(1, 90, PineappleEnum.MotorValueType.DEGREES, 4);

        sleep(1000);
        telemetry.addData("Encoder", motor.motorObject.getCurrentPosition());
        telemetry.update();
        sleep(1000);
    }
}


