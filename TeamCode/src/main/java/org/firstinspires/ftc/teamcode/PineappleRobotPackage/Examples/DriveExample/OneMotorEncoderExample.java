package org.firstinspires.ftc.teamcode.PineappleRobotPackage.Examples.DriveExample;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PSEnum;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PSMotor;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PSRobot;

/**
 * Created by young on 8/2/2017.
 */


@TeleOp(name = "PineEx-EncoderOneMotor", group = "Linear Opmode")
@Disabled

public class OneMotorEncoderExample extends LinearOpMode {
    PSRobot robot;

    PSMotor motor;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new PSRobot(this);

        motor = robot.motorHandler.newMotor("motor", 1, true , true, PSEnum.MotorType.NEV40);

        waitForStart();
        //motor.encoderDrive(1, 90, PSEnum.MotorValueType.DEGREES, 4);

        sleep(1000);
        telemetry.addData("Encoder", motor.motorObject.getCurrentPosition());
        telemetry.update();
        sleep(1000);
    }
}


