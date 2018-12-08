package org.firstinspires.ftc.teamcode.roverRuckus.Robot_r2;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import static java.lang.Math.atan2;
import static java.lang.Math.sqrt;
import static java.lang.Math.abs;

@TeleOp(name = "r2.Tele", group = "r2")
public class TeleOp_r2 extends Config {


    double cal = 0.0;

    @Override
    public void init() {
        config(this);
        lift.ratchetOn();
//        while (gyro..isCalibrating()) {
//            telemetry.addData("gyro", "cal");
//            telemetry.update();
//        }
        telemetry.addData("gyro", "ready");
        telemetry.update();
    }

    @Override
    public void loop() {
        //Drive switch drive and rotation sticks
        if (gamepad1.left_stick_button) {
            cal = gyro.getAngle();
            drive.thirdPerson = true;
        }else if(gamepad1.a){
            drive.thirdPerson = false;
        }
        telemetry.addData("gyro", gyro.getAngle());
        if(drive.thirdPerson) {
            robot.drive.mecanum.updateMecanumThirdPerson(gamepad1,  (gamepad1.right_stick_button) ? 1 : .5, Math.toRadians(gyro.getAngle() - cal));
        }else{
            robot.drive.mecanum.updateMecanum(gamepad1, (gamepad1.right_stick_button) ? 1.0 : .5);
        }

        //backup
        //robot.drive.mecanum.updateMecanum(gamepad1, 1.0);

        //collector
        collector.extension.setPower(-((abs(gamepad1.right_stick_y) > 0.70f) ? gamepad1.right_stick_y : gamepad2.right_stick_y));
        //collector.sweeperOn = (gamepad1.right_trigger > 0.9f) ? true : (gamepad1.left_trigger > 0.15f) ? false : collector.sweeperOn;
        collector.sweeper.setPower((collector.sweeperOn) ? 0.75 : (gamepad1.right_trigger > 0.15f) ? gamepad1.right_trigger : -(.65 * gamepad1.left_trigger));
        if (gamepad1.right_bumper) {
            collector.rampDown();
            collector.closeDoor();
        } else if (gamepad1.left_bumper) {
            collector.rampUp();
            collector.initDoor();
            collector.sweeperOn = false;
        }
        if (gamepad2.a) {
            collector.openDoor();
        }
        //Transfer
        transfer.shooterOn = (gamepad2.right_trigger > 0.15f) ? true : (gamepad2.left_trigger > 0.15f) ? false : transfer.shooterOn;
        transfer.shooter.setPower((transfer.shooterOn) ? -1.0 : gamepad2.left_trigger);
        transfer.feeder.setPower((gamepad2.right_bumper) ? -.5 : ((gamepad2.left_bumper) ? .5 : 0));
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
