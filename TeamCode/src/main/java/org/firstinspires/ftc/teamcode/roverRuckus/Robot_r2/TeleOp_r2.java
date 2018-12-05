package org.firstinspires.ftc.teamcode.roverRuckus.Robot_r2;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import static java.lang.Math.atan2;
import static java.lang.Math.sqrt;
import static java.lang.Math.abs;

@TeleOp(name = "r2.Tele", group = "r2")
public class TeleOp_r2 extends Config {

    @Override
    public void init() {
        config(this);
    }

    @Override
    public void loop() {
        //Drive switch drive and rotation sticks
        robot.drive.mecanum.updateMecanum(gamepad1, (gamepad1.right_bumper) ? .5 : 1.0);

        //backup
        //robot.drive.mecanum.updateMecanum(gamepad1, 1.0);

        //collector
        collector.extension.setPower(-((abs(gamepad1.right_stick_y) > 0.70f) ? gamepad1.right_stick_y : gamepad2.right_stick_y));
        collector.sweeperOn = (gamepad1.right_trigger > 0.9f) ? true : (gamepad1.left_trigger > 0.15f) ? false : collector.sweeperOn;
        collector.sweeper.setPower((collector.sweeperOn) ? 1.0 : (gamepad1.right_trigger > 0.15f) ? gamepad1.right_trigger : -gamepad1.left_trigger);
        if (gamepad2.x) {
            collector.rampDown();
        } else if (gamepad2.y) {
            collector.rampUp();
        }
        if (gamepad2.a && gamepad2.b) {
            collector.openDoor();
        } else if (gamepad2.a) {
            collector.closeDoor();
        }
        //Transfer
        transfer.shooterOn = (gamepad2.right_trigger > 0.15f) ? true : (gamepad2.left_trigger > 0.15f) ? false : transfer.shooterOn;
        transfer.shooter.setPower((transfer.shooterOn) ? 1.0 : -gamepad2.left_trigger);
        transfer.feeder.setPower((gamepad2.right_bumper) ? .5 : ((gamepad2.left_bumper) ? -.5 : 0));

        //Lift
        lift.extension.setPower((gamepad2.dpad_up) ? 1.0 : (gamepad2.dpad_down) ? -1.0 : 0.0);
        if (gamepad2.dpad_left) {

            lift.bridge.rotateL.setPosition(0);
        } else if (gamepad2.dpad_right) {
            lift.bridge.rotateL.setPosition(1);
        } else {
            lift.bridge.rotateL.off();
        } lift.drop.setPosition((gamepad1.b) ? lift.dropNormal : lift.dropInit);
        if (gamepad2.right_stick_button) {
            lift.ratchetOn();
        }
        lift.ratchetOff();
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
