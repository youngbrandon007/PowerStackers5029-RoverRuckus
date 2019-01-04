package org.firstinspires.ftc.teamcode.roverRuckus.Robot_r4;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.sqrt;

@TeleOp(name = "r4.Tele", group = "r4")
public class TeleOp_r4 extends Config_r4 {


    double cal = 0.0;

    @Override
    public void init() {
        config(this);
        lift.ratchetOn();
        telemetry.addData("gyro", "ready");
        telemetry.update();
    }

    @Override
    public void loop() {
        //Drive switch drive and rotation sticks
        if (gamepad1.left_stick_button) {
            cal = gyro.getHeading();
            drive.thirdPerson = true;
        }else if(gamepad1.a){
            drive.thirdPerson = false;
        }
        telemetry.addData("gyro", gyro.getHeading());
        if(drive.thirdPerson) {
            robot.drive.mecanum.updateMecanumThirdPerson(gamepad1,  (gamepad1.right_stick_button) ? 1.0 : .5, Math.toRadians(gyro.getHeading() - cal));
        }else{
            robot.drive.mecanum.updateMecanum(gamepad1, (gamepad1.right_stick_button) ? 1.0 : .5);
        }

        //backup
        //robot.drive.mecanum.updateMecanum(gamepad1, 1.0);

        //collector
        collector.extension.setPower(-((abs(gamepad1.right_stick_y) > 0.70f) ? gamepad1.right_stick_y : gamepad2.right_stick_y));

//        transfer.feeder.setPower(gamepad2.left_stick_x/2);
        //Lift
        lift.extension.setPower((gamepad2.dpad_up) ? 1.0 : (gamepad2.dpad_down) ? -1.0 : 0.0);
        if (!(abs(gamepad2.left_stick_x) < 0.6f && abs(gamepad2.left_stick_y) < 0.6f)) {
            telemetry.addData("bridge ", lift.bridge.setBridge2(Math.toDegrees(atan2(-gamepad2.left_stick_y, -gamepad2.left_stick_x))));
            telemetry.addData("bride.pos", Math.toDegrees(atan2(-gamepad2.left_stick_y, -gamepad2.left_stick_x)));

        }

        if (gamepad2.dpad_left) {

            lift.bridge.rotateL.setPosition(0);
        } else if (gamepad2.dpad_right) {
            lift.bridge.rotateL.setPosition(1);
        } else {
            //lift.bridge.rotateL.off();
        }
        lift.drop.setPosition((gamepad1.b) ? lift.dropNormal : lift.dropInit);

        if (gamepad2.right_stick_button) {
            lift.ratchetOn();
        } else if (gamepad2.left_stick_button) {
            lift.ratchetOff();
        }
    }

    private double getSpeed(Gamepad pad) {
        if (abs(pad.right_stick_x) < 0.15f
                && abs(pad.right_stick_y) < 0.15f) {
            return 0.0;
        } else {
            return sqrt((pad.right_stick_y * pad.right_stick_y)
                    + (pad.right_stick_x * pad.right_stick_x));
        }
    }
}
