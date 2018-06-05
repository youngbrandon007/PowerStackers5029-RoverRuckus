package org.firstinspires.ftc.teamcode.RelicRecoveryWorlds;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/**
 * Created by Brandon on 1/8/2018.
 */
@TeleOp(name = "Servo")
@Disabled
public class WorldServoPositionFinder extends WorldConfig {
    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }


//    @Override
//    public void runOpMode() throws InterruptedException {
//        config(this);
//
//        waitForStart();
//
////        boolean bClicked = false;
//        double collectorSpeed = 0;
//
//        while (opModeIsActive()) {
//            motorLift.setPower((gamepad2.dpad_up) ? -1 : (gamepad2.dpad_down) ? 1 : 0);
//
//            if (gamepad2.y) {
//                servoFlipL.setPosition(WorldConstants.flip.leftUp+collectorSpeed);
//                servoFlipR.setPosition(WorldConstants.flip.rightUp);
//            } else if (gamepad2.x) {
//                servoFlipL.setPosition(WorldConstants.flip.leftFlat+collectorSpeed);
//                servoFlipR.setPosition(WorldConstants.flip.rightFlat);
//            } else if (gamepad2.a) {
//                servoFlipL.setPosition(WorldConstants.flip.leftDown+collectorSpeed);
//                servoFlipR.setPosition(WorldConstants.flip.rightDown);
//            } else if (gamepad2.right_bumper) {
//                collectorSpeed+=0.0000005;
//           } else if (gamepad2.left_bumper) {
//                collectorSpeed-=0.0000005;
//            } else {
//                collectorSpeed+=0;
//            }
//            telemetry.addData("servoAdded", collectorSpeed);
//
//            telemetry.addData("servo Pos", servoFlipL.servoObject.getPosition());
//            telemetry.update();
//
//
//        }
//    }
}
