package org.firstinspires.ftc.teamcode.RelicRecoveryFinalRobot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/**
 * Created by Brandon on 1/8/2018.
 */
@TeleOp(name = "Servo")
@Disabled
public class ServoFinder extends Config {
    @Override
    public void runOpMode() throws InterruptedException {
        config(this);

        waitForStart();

//        boolean bClicked = false;
        double collectorSpeed = 0;

        while (opModeIsActive()) {
            motorLift.setPower((gamepad2.dpad_up) ? -1 : (gamepad2.dpad_down) ? 1 : 0);

            if (gamepad2.y) {
                servoFlipL.setPosition(Constants.flip.leftUp+collectorSpeed);
                servoFlipR.setPosition(Constants.flip.rightUp);
            } else if (gamepad2.x) {
                servoFlipL.setPosition(Constants.flip.leftFlat+collectorSpeed);
                servoFlipR.setPosition(Constants.flip.rightFlat);
            } else if (gamepad2.a) {
                servoFlipL.setPosition(Constants.flip.leftDown+collectorSpeed);
                servoFlipR.setPosition(Constants.flip.rightDown);
            } else if (gamepad2.right_bumper) {
                collectorSpeed+=0.0000005;
           } else if (gamepad2.left_bumper) {
                collectorSpeed-=0.0000005;
            } else {
                collectorSpeed+=0;
            }
            telemetry.addData("servoAdded", collectorSpeed);

            telemetry.addData("servo Pos", servoFlipL.servoObject.getPosition());
            telemetry.update();


        }
    }
}
