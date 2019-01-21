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
        if (gamepad1.x) {
            cal = gyro.getHeading();
            drive.thirdPerson = true;
        }else if(gamepad1.a){
            drive.thirdPerson = false;
        }
        telemetry.addData("gyro", gyro.getHeading());
        if(drive.thirdPerson) {
            robot.drive.mecanum.updateMecanumThirdPerson(gamepad1,  (gamepad1.right_stick_button) ? 1.0 : .5, -Math.toRadians(gyro.getHeading() - cal));
        }else{
            robot.drive.mecanum.updateMecanum(gamepad1, (gamepad1.right_stick_button) ? 1.0 : .5);
        }


        //collector
        collector.extension.setPower((gamepad1.right_bumper) ? 1 : ((gamepad1.left_bumper)?-1:0));
        collector.extension.setPower((gamepad2.right_bumper) ? 1 : ((gamepad2.left_bumper)?-1:0));

        collector.setCollectorPower(gamepad1.right_trigger-gamepad1.left_trigger);

        //TODO collector rotate servo
        if (gamepad1.right_bumper){
            collector.collectorRotate.setPosition(0.2);
        } else if (gamepad1.left_bumper){
            collector.collectorRotate.setPosition(0.5);
        }
        //Lift
        lift.extension.setPower((gamepad2.dpad_up) ? -1.0 : (gamepad2.dpad_down) ? 1.0 : 0.0);
        if (!(abs(gamepad2.left_stick_x) < 0.6f && abs(gamepad2.left_stick_y) < 0.6f)) {
            telemetry.addData("bridge ", lift.bridge.setBridge2(Math.toDegrees(atan2(-gamepad2.left_stick_y, -gamepad2.left_stick_x))));
            telemetry.addData("bride.pos", Math.toDegrees(atan2(-gamepad2.left_stick_y, -gamepad2.left_stick_x)));

        }

        if (gamepad2.dpad_up) {

            lift.bridge.rotateL.setPosition(lift.bridge.rotateL.getCachedPosition()+0.0005);
        } else if (gamepad2.dpad_down) {
            lift.bridge.rotateL.setPosition(lift.bridge.rotateL.getCachedPosition()-0.0005);
        }
        
        if (gamepad2.right_stick_button) {
            lift.ratchetOn();
        } else if (gamepad2.left_stick_button) {
            lift.ratchetOff();
        }

        lift.bridge.doorServo.setPosition((gamepad1.b) ? lift.dropNormal : lift.dropInit);

    }
}
