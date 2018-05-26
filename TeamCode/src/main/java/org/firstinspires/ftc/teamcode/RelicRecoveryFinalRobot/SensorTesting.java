package org.firstinspires.ftc.teamcode.RelicRecoveryFinalRobot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleEnum;

/**
 * Created by young on 2/9/2018.
 */
@Autonomous(name = "Sensor Test", group = "")
public class SensorTesting extends Config {
    @Override
    public void runOpMode() throws InterruptedException {
        config(this);
        glyphColor.enableLed(true);

        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("LIFT", motorLift.getEncoderPosition());
            telemetry.addData("LB", limitLeftBack.getState());
            telemetry.addData("LS", limitLeftSide.getState());
            telemetry.addData("RB", limitRightBack.getState());
            telemetry.addData("RS", limitRightSide.getState());
            telemetry.addData("Glyph Optic", opticalGlyph.getLightDetected());
            telemetry.addData("color R", glyphColor.red());
            telemetry.addData("color G", glyphColor.green());
            telemetry.addData("color B", glyphColor.blue());
            telemetry.addData("Glyph color", (glyphColor.red() + glyphColor.blue() + glyphColor.green())/3);
            telemetry.addData("color JEWELR  R", csJewelRight.getValue(PineappleEnum.PineappleSensorEnum.CSRED));
            telemetry.addData("color JEWELR G", csJewelRight.getValue(PineappleEnum.PineappleSensorEnum.CSGREEN));
            telemetry.addData("color JEWELR B", csJewelRight.getValue(PineappleEnum.PineappleSensorEnum.CSBLUE));
            telemetry.addData("color JEWELL  R", csJewelLeft.getValue(PineappleEnum.PineappleSensorEnum.CSRED));
            telemetry.addData("color JEWELL G", csJewelLeft.getValue(PineappleEnum.PineappleSensorEnum.CSGREEN));
            telemetry.addData("color JEWELL B", csJewelLeft.getValue(PineappleEnum.PineappleSensorEnum.CSBLUE));
            telemetry.update();
        }
    }
}
