package org.firstinspires.ftc.teamcode.RelicRecoveryWorlds;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PSEnum;

import java.util.Locale;

/**
 * Created by young on 2/9/2018.
 */
@Autonomous(name = "Sensor Test", group = "")
public class WorldSensorCheck extends WorldConfig {
    @Override
    public void init() {
        config(this);
//        glyphColor.enableLed(true);
        opticalGlyph.enableLed(true);
    }

    @Override
    public void loop() {
        telemetry.addData("LIFT", motorLift.getEncoderPosition());
        telemetry.addData("DFR", driveFrontRight.getEncoderPosition());
        telemetry.addData("DFL", driveFrontLeft.getEncoderPosition());
        telemetry.addData("DBR", driveBackRight.getEncoderPosition());
        telemetry.addData("DBL", driveBackLeft.getEncoderPosition());
            telemetry.addData("LB", limitLeftBack.getState());
            telemetry.addData("LS", limitLeftSide.getState());
            telemetry.addData("RB", limitRightBack.getState());
            telemetry.addData("RS", limitRightSide.getState());
        telemetry.addData("Distance (cm)",
                String.format(Locale.US, "%.02f", glyphDist.getDistance(DistanceUnit.CM)));
        telemetry.addData("Alpha", backGlyphColor.alpha());
        telemetry.addData("Red  ", backGlyphColor.red());
        telemetry.addData("Green", backGlyphColor.green());
        telemetry.addData("Blue ", backGlyphColor.blue());
        telemetry.addData("Distance (cm)",
                String.format(Locale.US, "%.02f", collectDist.getDistance(DistanceUnit.CM)));
        telemetry.addData("Alpha", collectColor.alpha());
        telemetry.addData("Red  ", collectColor.red());
        telemetry.addData("Green", collectColor.green());
        telemetry.addData("Blue ", collectColor.blue());
        telemetry.addData("ODS", opticalGlyph.getLightDetected());
        telemetry.addData("ODS-RAW", opticalGlyph.getRawLightDetected());
        telemetry.addData("ODSR", opticalRight.getLightDetected());
        telemetry.addData("ODSR-RAW", opticalRight.getRawLightDetected());
        telemetry.addData("ODSL", opticalLeft.getLightDetected());
        telemetry.addData("ODSL-RAW", opticalLeft.getRawLightDetected());
            //telemetry.addData("Glyph Optic", opticalGlyph.getLightDetected());
            //telemetry.addData("color R", glyphColor.red());
            //telemetry.addData("color G", glyphColor.green());
            //telemetry.addData("color B", glyphColor.blue());
            //telemetry.addData("Glyph color", (glyphColor.red() + glyphColor.blue() + glyphColor.green())/3);
            telemetry.addData("color JEWELR  R", csJewelRight.getValue(PSEnum.PineappleSensorEnum.CSRED));
            telemetry.addData("color JEWELR G", csJewelRight.getValue(PSEnum.PineappleSensorEnum.CSGREEN));
            telemetry.addData("color JEWELR B", csJewelRight.getValue(PSEnum.PineappleSensorEnum.CSBLUE));
            telemetry.addData("color JEWELL  R", csJewelLeft.getValue(PSEnum.PineappleSensorEnum.CSRED));
            telemetry.addData("color JEWELL G", csJewelLeft.getValue(PSEnum.PineappleSensorEnum.CSGREEN));
            telemetry.addData("color JEWELL B", csJewelLeft.getValue(PSEnum.PineappleSensorEnum.CSBLUE));
    }


//    @Override
//    public void runOpMode() throws InterruptedException {
//        config(this);
//        glyphColor.enableLed(true);
//
//        waitForStart();
//        while(opModeIsActive()){
//            telemetry.addData("LIFT", motorLift.getEncoderPosition());
//            telemetry.addData("LB", limitLeftBack.getState());
//            telemetry.addData("LS", limitLeftSide.getState());
//            telemetry.addData("RB", limitRightBack.getState());
//            telemetry.addData("RS", limitRightSide.getState());
//            telemetry.addData("Glyph Optic", opticalGlyph.getLightDetected());
//            telemetry.addData("color R", glyphColor.red());
//            telemetry.addData("color G", glyphColor.green());
//            telemetry.addData("color B", glyphColor.blue());
//            telemetry.addData("Glyph color", (glyphColor.red() + glyphColor.blue() + glyphColor.green())/3);
//            telemetry.addData("color JEWELR  R", csJewelRight.getValue(PSEnum.PineappleSensorEnum.CSRED));
//            telemetry.addData("color JEWELR G", csJewelRight.getValue(PSEnum.PineappleSensorEnum.CSGREEN));
//            telemetry.addData("color JEWELR B", csJewelRight.getValue(PSEnum.PineappleSensorEnum.CSBLUE));
//            telemetry.addData("color JEWELL  R", csJewelLeft.getValue(PSEnum.PineappleSensorEnum.CSRED));
//            telemetry.addData("color JEWELL G", csJewelLeft.getValue(PSEnum.PineappleSensorEnum.CSGREEN));
//            telemetry.addData("color JEWELL B", csJewelLeft.getValue(PSEnum.PineappleSensorEnum.CSBLUE));
//            telemetry.update();
//        }
//    }
}
