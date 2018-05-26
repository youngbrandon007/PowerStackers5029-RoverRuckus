package org.firstinspires.ftc.teamcode.RelicRecoveryFinalRobot;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Old_Robots.RelicRecovery.RelicRecoveryOfficalFile.RelicResources.RelicRecoveryEnums;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleEnum;

import static org.firstinspires.ftc.teamcode.RelicRecoveryFinalRobot.Constants.auto.jewel.jewelHitSide.LEFT;
import static org.firstinspires.ftc.teamcode.RelicRecoveryFinalRobot.Constants.auto.jewel.jewelHitSide.RIGHT;
import static org.firstinspires.ftc.teamcode.RelicRecoveryFinalRobot.Constants.auto.jewel.jewelState.BLUE_RED;
import static org.firstinspires.ftc.teamcode.RelicRecoveryFinalRobot.Constants.auto.jewel.jewelState.NON_NON;
import static org.firstinspires.ftc.teamcode.RelicRecoveryFinalRobot.Constants.auto.jewel.jewelState.RED_BLUE;

/**
 * Created by Brandon on 1/8/2018.
 */
@Autonomous(name = "AUTO Van")
@Disabled
public class AutoVan extends Config {

    @Override
    public void runOpMode() throws InterruptedException {
        config(this);

        waitForStart();
        jewelCSLEDON();
        int x =  jewelDown();
        Thread.sleep(x);
        x =  jewelHit();
        Thread.sleep(x);
        x =  jewelUp();
        Thread.sleep(x);
    }
    public void jewelCSLEDON() {
        csJewelRight.getValue(PineappleEnum.PineappleSensorEnum.CSLEDON);
        csJewelLeft.getValue(PineappleEnum.PineappleSensorEnum.CSLEDON);

    }

    public void jewelCSLEDOFF() {
        csJewelRight.getValue(PineappleEnum.PineappleSensorEnum.CSLEDOFF);
        csJewelLeft.getValue(PineappleEnum.PineappleSensorEnum.CSLEDOFF);

    }

    public int jewelDown() {
        servoJewel.setPosition(Constants.auto.jewel.JEWELDOWN);
        servoJewelHit.setPosition(Constants.auto.jewel.JEWELHITCENTER);
        return Constants.auto.jewel.JEWELDOWNMILI;
    }

    public int jewelUp() {
        servoJewel.setPosition(Constants.auto.jewel.JEWELUP);
        return Constants.auto.jewel.JEWELUPMILI;
    }

    public int jewelHit() {
        switch (jewelHitSideSimple()) {
            case RIGHT:
                servoJewelHit.setPosition(Constants.auto.jewel.JEWELHITRIGHT);
                return Constants.auto.jewel.JEWELHITMILI;
            case LEFT:
                servoJewelHit.setPosition(Constants.auto.jewel.JEWELHITLEFT);
                return Constants.auto.jewel.JEWELHITMILI;
            case NONE:
                return 0;
            default:
                return 0;
        }

    }

    private Constants.auto.jewel.jewelState getLeftCSJewelState() {
        if (csJewelLeft.getValue(PineappleEnum.PineappleSensorEnum.CSBLUE) > csJewelLeft.getValue(PineappleEnum.PineappleSensorEnum.CSRED)) {
            return BLUE_RED;
        } else if (csJewelLeft.getValue(PineappleEnum.PineappleSensorEnum.CSBLUE) < csJewelLeft.getValue(PineappleEnum.PineappleSensorEnum.CSRED)) {
            return RED_BLUE;
        } else {
            return NON_NON;
        }
    }

    private Constants.auto.jewel.jewelState getRightCSJewelState() {

        if (csJewelRight.getValue(PineappleEnum.PineappleSensorEnum.CSBLUE) < csJewelRight.getValue(PineappleEnum.PineappleSensorEnum.CSRED)) {
            return BLUE_RED;
        } else if (csJewelRight.getValue(PineappleEnum.PineappleSensorEnum.CSBLUE) > csJewelRight.getValue(PineappleEnum.PineappleSensorEnum.CSRED)) {
            return RED_BLUE;
        } else {
            return NON_NON;
        }
    }
    public Constants.auto.jewel.jewelHitSide jewelHitSideSimple() {
        Constants.auto.jewel.jewelState left = getLeftCSJewelState();
        Constants.auto.jewel.jewelState right = getRightCSJewelState();
        Constants.auto.jewel.jewelState state;

        if (left == right) {
            state = left;
        } else if (left == NON_NON&&right!=NON_NON){
            state = right;
        }else if (right == NON_NON&&left!=NON_NON){
            state = left;
        } else {
            state = NON_NON;
        }
        return (switchColor == RelicRecoveryEnums.AutoColor.RED) ? (state == RED_BLUE) ? RIGHT : LEFT : (state == RED_BLUE) ? LEFT : RIGHT;

    }
}
