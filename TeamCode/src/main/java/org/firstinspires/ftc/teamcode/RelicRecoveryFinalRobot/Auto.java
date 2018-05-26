package org.firstinspires.ftc.teamcode.RelicRecoveryFinalRobot;


import android.graphics.Bitmap;
import android.graphics.Path;

import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraCalibration;
import com.vuforia.EyewearUserCalibrator;
import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Tool;
import com.vuforia.Vec3F;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.android.dx.cf.cst.ConstantTags;
import org.firstinspires.ftc.teamcode.Old_Robots.RelicRecovery.RelicRecoveryOfficalFile.RelicResources.RelicRecoveryEnums;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleEnum;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleRobotConstants;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.Vuforia.PineappleRelicRecoveryVuforia;
import org.firstinspires.ftc.teamcode.RelicRecoveryFinalRobot.Constants.auto.autoGlyph.column;
import org.firstinspires.ftc.teamcode.RelicRecoveryFinalRobot.Constants.auto.autoGlyph.glyph;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.nio.ByteBuffer;
import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.Vuforia.PineappleRelicRecoveryVuforia.SaveImage;
import static org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.Vuforia.PineappleRelicRecoveryVuforia.matToBitmap;
import static org.firstinspires.ftc.teamcode.RelicRecoveryFinalRobot.Constants.auto.autoGlyph.glyph.BROWN;
import static org.firstinspires.ftc.teamcode.RelicRecoveryFinalRobot.Constants.auto.autoGlyph.glyph.GREY;
import static org.firstinspires.ftc.teamcode.RelicRecoveryFinalRobot.Constants.auto.autoGlyph.glyph.NONE;
import static org.firstinspires.ftc.teamcode.RelicRecoveryFinalRobot.Constants.auto.jewel.jewelHitSide.LEFT;
import static org.firstinspires.ftc.teamcode.RelicRecoveryFinalRobot.Constants.auto.jewel.jewelHitSide.RIGHT;
import static org.firstinspires.ftc.teamcode.RelicRecoveryFinalRobot.Constants.auto.jewel.jewelState.BLUE_RED;
import static org.firstinspires.ftc.teamcode.RelicRecoveryFinalRobot.Constants.auto.jewel.jewelState.NON_NON;
import static org.firstinspires.ftc.teamcode.RelicRecoveryFinalRobot.Constants.auto.jewel.jewelState.RED_BLUE;

/**
 * Created by Brandon on 1/8/2018.
 */
@Autonomous(name = "AUTO")
public class Auto extends Config {

    enum InitEnum {
        HARDWAREINIT, GYRO, VUFORIA, LOOP
    }

    enum AutoEnum {
        WAIT,
        JEWELS, JEWELDOWN, JEWELHIT, JEWELUP,
        ALIGN, ALIGNDRIVEOFFPLATFORM, ALIGNTURN, ALIGNDRIVEINTOCRYPTO,
        KEYCOLUMNSET,
        GLYPH, GLYPHSTRAFFTOALIGN, GLYPHLIFT, GLYPHPLACE, GLYPHINTOBOX, GLYPHLIFTDOWN, GLYPHPLACERESET,
        COLLECT, COLLECTDRIVEBACKFROMCRYPTO, COLLECTSTRAFFTOCENTER, COLLECTSTARTTRACKING, COLLECTGOTOPIT, COLLECTGLYPHS, COLLECTFINISHCOLLECTING, COLLECTRETRACESTEPS, COLLECTPROCESSFORPLACING, CHANGESETPOINT
    }

    ElapsedTime wait = new ElapsedTime();
    ElapsedTime collectorRPMTimer = new ElapsedTime();

    AutoEnum auto = AutoEnum.WAIT;
    InitEnum init = InitEnum.HARDWAREINIT;
    RelicRecoveryVuMark keyColumn = RelicRecoveryVuMark.UNKNOWN;
    RelicRecoveryVuMark targetColumn = RelicRecoveryVuMark.CENTER;
    RelicRecoveryVuMark previousColumn = RelicRecoveryVuMark.CENTER;
    int columnNumber = 1;

    glyph frontGlyph = NONE;
    glyph backGlyph = NONE;

    boolean usingRightArm = true;

    VuforiaLocalizer vuforia;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;
    VuforiaTrackableDefaultListener listener;
    glyph startGlyph;
    boolean vuforiaInitialized = false;
    boolean imageVisible = false;
    boolean jewelScanned = false;
    boolean ready = false;

    double direction = 0.0;
    double distance = 0.0;

    int oneGlyphCollected = 0;

    int lift = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        config(this);

        startGlyph = getGlyph();
        //INIT LOOP
        while (!opModeIsActive() && !isStopRequested()) {
            //Switch Board Loading
            loadSwitchBoard();


            //Telemetry
            telemetry.addData("INIT", init);
            telemetry.addLine();
            displaySwitchBorad((startGlyph == GREY) ? true : false);
            telemetry.addLine();
            telemetry.addLine(FontFormating.getMark(calibration_complete) + "GYRO CALIBRATION");
            telemetry.addLine(FontFormating.getMark(vuforiaInitialized) + "VUFORIA");
            telemetry.addLine(FontFormating.getMark(imageVisible) + "IMAGE VISIBLE-" + keyColumn);
            telemetry.addLine(FontFormating.getMark(jewelScanned) + "JEWELS-" + jewelState);
            telemetry.addData("Glyph", getGlyph());

            switch (init) {
                case HARDWAREINIT:
                    servoFlipL.setPosition(Constants.flip.leftFlat);
                    servoFlipR.setPosition(Constants.flip.rightFlat);
                    servoFlipL.setPosition(Constants.flip.leftDown);
                    servoFlipR.setPosition(Constants.flip.rightDown);
                    glyphColor.enableLed(true);
                    init = InitEnum.GYRO;
                    break;
                case GYRO:
                    calibration_complete = !navx_device.isCalibrating();
                    if (!calibration_complete) {
                    } else {
                        navx_device.zeroYaw();
                        yawPIDResult = new navXPIDController.PIDResult();
                        init = InitEnum.VUFORIA;
                    }
                    break;
                case VUFORIA:
                    int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
                    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
                    parameters.vuforiaLicenseKey = "AdB8VB7/////AAAAGcfBp9I80URFkfBQFUyM+ptmQXBAMGx0svJKz7QE2nm20mBc/zI5sZNHfuP/ziIm+sYnO7fvPqUbFs8QWjRyXVEDmW4mMj+S+l+yaYRkpGZ/pmHyXiDb4aemHx0m70BulMNIce4+NVaCW5S/5BWNNev/AU0P+uWnHYuKNWzD2dPaRuprC4R6b/DgD1zeio1xlssYb9in9mfzn76gChOrE5B0ql6Q9FiHC5cTdacq2lKjm5nlkTiXz1e2jhVK3SddGoqM4FQ3mFks7/A88hFzlPfIIk45K2Lh7GvcVjuIiqNj5mTLaZJVqlsKdTQnKS4trJcc1YV9sjdbmh1agtn1UePy91fDj9uWSBdXvpIowv4B";
                    parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
                    vuforia = ClassFactory.createVuforiaLocalizer(parameters);
                    vuforia.setFrameQueueCapacity(1);
                    Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
                    relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
                    relicTemplate = relicTrackables.get(0);
                    relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
                    listener = (VuforiaTrackableDefaultListener) relicTemplate.getListener();
                    relicTrackables.activate();
                    vuforiaInitialized = true;
                    ready = true;
                    init = InitEnum.LOOP;
                    break;
                case LOOP:
                    if (listener.getPose() != null) {
                        imageVisible = true;
                        keyColumn = RelicRecoveryVuMark.from(relicTemplate);
                        jewelState = getJewelConfig(PineappleRelicRecoveryVuforia.getImageFromFrame(vuforia.getFrameQueue().take(), PIXEL_FORMAT.RGB565), listener, vuforia.getCameraCalibration(), telemetry);
                    } else {
                        imageVisible = false;
                    }
                    telemetry.addData("Gyro", navx_device.getYaw());
                    break;
            }

            telemetry.update();
        }
        waitForStart();
        servoAlignRight.setPosition(Constants.alignment.ALIGNRIGHTUP);
        servoAlignLeft.setPosition(Constants.alignment.ALIGNLEFTUP);
        wait.reset();
        double TARGETANGLE = 0.0;
        double PIDrotationOut = 0;
        boolean PIDonTarget = false;
        int trackBack = 0;
        frontGlyph = startGlyph;
        collectorRPMTimer.reset();
        double prervTime = collectorRPMTimer.milliseconds();
        double prervPos = motorCollectLeft.getEncoderPosition();
        int x = 0;
        //MAIN LOOP
        while (opModeIsActive()) {
            //Always On Telemetry
            telemetry.addData("AUTO: ", auto);
            double encoderDif = motorCollectLeft.getEncoderPosition() - prervPos;
            double timeDif = collectorRPMTimer.milliseconds() - prervTime;
            double RPM = Math.abs((60000 / timeDif) * encoderDif);
            telemetry.addData("Collector RPM", RPM);
            if (switchPID) {
                if (yawPIDController.isNewUpdateAvailable(yawPIDResult)) {
                    if (yawPIDResult.isOnTarget()) {
                        PIDonTarget = true;
                        PIDrotationOut = 0.0;
                    } else {
                        PIDrotationOut = yawPIDResult.getOutput();
                        PIDonTarget = false;
                    }
                }
            } else {
                double gyroOffset = navx_device.getYaw() - TARGETANGLE;
                while (gyroOffset > 180 || gyroOffset < -180) {
                    gyroOffset += (gyroOffset > 180) ? -360 : (gyroOffset < -180) ? 360 : 0;
                }

                if (gyroOffset > 2) {
                    PIDrotationOut = -.2;
                    PIDonTarget = false;
                } else if (gyroOffset < -2) {
                    PIDrotationOut = .2;
                    PIDonTarget = false;
                } else {
                    PIDrotationOut = 0;
                    PIDonTarget = true;
                }
            }
            telemetry.addLine("GYRO→TARGET: " + navx_device.getYaw() + "→" + TARGETANGLE);

            switch (auto) {
                case WAIT:
                    if (!switchDelayEnabled || wait.seconds() >= slideDelay) {
                        auto = AutoEnum.JEWELS;
                    }
                    break;
                case JEWELS:
                    if (!switchJewels) {
                        auto = AutoEnum.ALIGN;
                    } else {
                        auto = AutoEnum.JEWELDOWN;
                        wait.reset();
                    }
                    break;
                case JEWELDOWN:
                    servoJewelHit.setPosition(Constants.auto.jewel.JEWELHITCENTER);

                    if (wait.milliseconds() > 200) {
                        servoJewel.setPosition(Constants.auto.jewel.JEWELDOWN);

                        if (wait.milliseconds() > 400) {
                            auto = AutoEnum.JEWELHIT;
                            wait.reset();
                        }
                    }
                    jewelCSLEDON();
                    break;
                case JEWELHIT:
                    if (wait.milliseconds() > jewelHit()) {
                        auto = AutoEnum.JEWELUP;
                        wait.reset();
                    }
                    break;
                case JEWELUP:
                    if (wait.milliseconds() > jewelUp() / 2) {
                            auto = AutoEnum.ALIGN;
                            wait.reset();

                    }
                    jewelCSLEDOFF();

                    break;
                case ALIGN:

                    auto = AutoEnum.KEYCOLUMNSET;
                    break;
                case KEYCOLUMNSET:
                    if (keyColumn != RelicRecoveryVuMark.UNKNOWN) {
                        targetColumn = keyColumn;
                    }
                    columnNumber = columnNumber(targetColumn);
                    auto = AutoEnum.ALIGNDRIVEOFFPLATFORM;
                    resetEncoders();
                    break;

                case ALIGNDRIVEOFFPLATFORM:
                    servoJewelHit.setPosition(Constants.auto.jewel.JEWELHITLEFT);
                    robotHandler.drive.mecanum.setMecanum(Math.toRadians(Constants.auto.aligning.AlignDriveOffPlatformDirection[colorPositionInt]), 0.5, PIDrotationOut, 1.0);
                    if (traveledEncoderTicks(Constants.drive.countsPerInches(Constants.auto.aligning.AlignDrivingOffPlatformEncoder[colorPositionInt][columnNumber]))) {
                        robotHandler.drive.stop();
                        TARGETANGLE = Constants.auto.aligning.AlignTurnAngle[colorPositionInt];
                        yawPIDController.setSetpoint(TARGETANGLE);
                        auto = AutoEnum.ALIGNTURN;
                    }
                    break;
                case ALIGNTURN:
                    robotHandler.drive.mecanum.setMecanum(0.0, 0.0, PIDrotationOut, 1.0);
                    servoAlignRight.setPosition(Constants.auto.aligning.AlignArmPosition[colorPositionInt][0][columnNumber]);
                    servoAlignLeft.setPosition(Constants.auto.aligning.AlignArmPosition[colorPositionInt][1][columnNumber]);
                    usingRightArm = Constants.auto.aligning.AlignSwitchClicked[colorPositionInt][0][columnNumber];
                    if (PIDonTarget) {
                        robotHandler.drive.stop();
                        auto = AutoEnum.ALIGNDRIVEINTOCRYPTO;
                        wait.reset();
                    }
                    break;
                case ALIGNDRIVEINTOCRYPTO:
//                    switchPID = false;
                    PIDrotationOut = 0;
                    robotHandler.drive.mecanum.setMecanum(Math.toRadians(270), .5, PIDrotationOut, 1.0);
                    if ((limitRightBack.getState() && usingRightArm) || (limitLeftBack.getState() && !usingRightArm) || wait.milliseconds() > 5000) {
                        robotHandler.drive.stop();
                        auto = AutoEnum.GLYPH;
                        motorCollectRight.setPower(0);
                        motorCollectLeft.setPower(0);
                        robotHandler.drive.mecanum.setMecanum(Math.toRadians(90), 0.4, PIDrotationOut, 1.0);
                    }
                    break;
                case GLYPH:
                    auto = AutoEnum.GLYPHSTRAFFTOALIGN;
                    wait.reset();
                    break;
                case GLYPHSTRAFFTOALIGN:
                    switchPID = false;
                    robotHandler.drive.mecanum.setMecanum(Math.toRadians((usingRightArm) ? 180 : 0), .7, PIDrotationOut, 1.0);
                    if ((limitRightSide.getState() && usingRightArm) || (limitLeftSide.getState() && !usingRightArm) || wait.milliseconds() > 5000) {
                        robotHandler.drive.stop();
                        auto = AutoEnum.GLYPHPLACE;
//                        robotHandler.drive.mecanum.setMecanum(Math.toRadians(90), 0.4, PIDrotationOut, 1.0);
                        robotHandler.drive.mecanum.setMecanum(Math.toRadians((usingRightArm) ? 0 : 180), 0.6, PIDrotationOut, 1.0);
                        wait.reset();
                    }
                    break;

                case GLYPHPLACE:

                    robotHandler.drive. mecanum.setMecanum(0, 0, PIDrotationOut, 1.0);


                    servoFlipL.setPosition(Constants.flip.leftUp);
                    servoFlipR.setPosition(Constants.flip.rightUp);
                    servoAlignLeft.setPosition(Constants.alignment.ALIGNLEFTUP);
                    servoAlignRight.setPosition(Constants.alignment.ALIGNRIGHTUP);
                    if (wait.milliseconds() > 1000) {
                        //addGlyphsToColumn(COLUMN, FIRST GLYPH COLOR, SECOND GLYPH COLOR);
                        addGlyphsToColumn(targetColumn, frontGlyph, backGlyph);

                        auto = AutoEnum.GLYPHINTOBOX;
                        wait.reset();
                    }
                    break;
                case GLYPHINTOBOX:
                    robotHandler.drive.mecanum.setMecanum(Math.toRadians(270), 0.4, PIDrotationOut, 1.0);
                    if (wait.milliseconds() > 1000) {
                        //addGlyphsToColumn(COLUMN, FIRST GLYPH COLOR, SECOND GLYPH COLOR);
                        robotHandler.drive.mecanum.setMecanum(Math.toRadians(90), 1.0, PIDrotationOut, 1.0);
                        auto = AutoEnum.GLYPHPLACERESET;
                        wait.reset();
                    }
                    break;
                case GLYPHPLACERESET:
                    robotHandler.drive. mecanum.setMecanum(0, 0, PIDrotationOut, 1.0);
                    lift = 0;
                    robotHandler.drive.stop();
                    servoFlipL.setPosition(Constants.flip.leftDown);
                    servoFlipR.setPosition(Constants.flip.rightDown);
                    if (wait.milliseconds() > 500) {
                        auto = AutoEnum.COLLECT;
                    }
                    break;
                case COLLECT:
                    resetEncoders();

                    auto = AutoEnum.COLLECTGOTOPIT;
                    break;
                case COLLECTDRIVEBACKFROMCRYPTO:
                    break;
                case COLLECTSTRAFFTOCENTER:
                    break;
                case COLLECTSTARTTRACKING:
                    break;
                case COLLECTGOTOPIT:
                    robotHandler.drive.mecanum.setMecanum(Math.toRadians(90), 1.0, PIDrotationOut, 1.0);
                    if (traveledEncoderTicks(Constants.drive.countsPerInches(Constants.auto.aligning.CollectDistToPit))) {
                        robotHandler.drive.stop();
                        wait.reset();
                        auto = AutoEnum.COLLECTGLYPHS;
                        servoFlipL.setPosition(Constants.flip.leftDown);
                        servoFlipR.setPosition(Constants.flip.rightDown);
                        oneGlyphCollected = 0;
                    }
                    break;
                case COLLECTGLYPHS:
                    robotHandler.drive.mecanum.setMecanum(Math.toRadians(90), 0.6, PIDrotationOut, 1.0);

                    motorCollectRight.setPower(1.0);
                    motorCollectLeft.setPower(-1.0);
////                     if (RPM<0) {
////                        x++;
////
////                     }
////                    if (x>1) {
////                        x--;
////                        motorCollectRight.setPower(-1.0);
////                        motorCollectLeft.setPower(1.0);
////                    }
                    if (traveledEncoderTicks(Constants.drive.countsPerInches(40))) {
                        robotHandler.drive.stop();
                        auto = AutoEnum.COLLECTFINISHCOLLECTING;
                        wait.reset();
                        backGlyph = getGlyph();
                    } else if ((oneGlyphCollected == 1 && wait.milliseconds() > Constants.auto.aligning.collectDriveIntoPitTime) || opticalGlyph.getLightDetected() > 0.0) {
                        robotHandler.drive.stop();
                        auto = AutoEnum.COLLECTFINISHCOLLECTING;
                        wait.reset();
                        backGlyph = getGlyph();
                    } else if((glyphColor.red() + glyphColor.green() + glyphColor.blue())/3.0 > 0){
                        if(oneGlyphCollected == 0) {
                            wait.reset();
                            oneGlyphCollected = 1;
                        }
                    }else {

                        robotHandler.drive.mecanum.setMecanum(Math.toRadians(90), 0.4, PIDrotationOut, 1.0);
                    }

//                    if (wait.milliseconds() > Constants.auto.aligning.collectDriveIntoPitTime || opticalGlyph.getLightDetected() > 0.0){
//                        robotHandler.drive.stop();
//                        auto = AutoEnum.COLLECTFINISHCOLLECTING;
//                        wait.reset();
//                        backGlyph = getGlyph();
//                    }

                    break;
                case COLLECTFINISHCOLLECTING:
                        trackBack = getTraveledEncoderTicks();
                        resetEncoders();
                        auto = AutoEnum.COLLECTRETRACESTEPS;


                        wait.reset();
                    break;
                case COLLECTRETRACESTEPS:

                    robotHandler.drive.mecanum.setMecanum(Math.toRadians(270), .6, PIDrotationOut, 1.0);
                    if (traveledEncoderTicks(trackBack - Constants.drive.countsPerInches(Constants.auto.aligning.GlyphDistanceToCrypto))) {
                        robotHandler.drive.stop();
                        auto = AutoEnum.CHANGESETPOINT;
                        resetEncoders();
                        frontGlyph = getGlyph();
                        servoFlipL.setPosition(Constants.flip.leftFlat);
                        servoFlipR.setPosition(Constants.flip.rightFlat);
                        motorCollectRight.setPower(-1.0);
                        motorCollectLeft.setPower(1.0);
                    }
                    break;
                case CHANGESETPOINT:
                    previousColumn = targetColumn;
                    targetColumn = getColumn(frontGlyph, backGlyph);
                    columnNumber = columnNumber(targetColumn);
                    int alignment;
                    double previousColumnNumber = columnNumber(previousColumn);
                    distance = (columnNumber - previousColumnNumber) * Constants.auto.aligning.columnStraffDistance;
                    alignment = (columnNumber == 0) ? 0 : (columnNumber == 2) ? 1 : (BOX[0][0] == NONE) ? 0 : 1;
                    distance += (alignment == 1) ? Constants.auto.aligning.alignStraffDistance : -Constants.auto.aligning.alignStraffDistance;
                    if (distance > 0) {
                        direction = 180;
                    } else if (distance < 0) {
                        direction = 0;
                    }
                    servoAlignRight.setPosition((alignment == 0) ? Constants.alignment.ALIGNRIGHTDOWN : Constants.alignment.ALIGNRIGHTUP);
                    servoAlignLeft.setPosition((alignment == 1) ? Constants.alignment.ALIGNLEFTDOWN : Constants.alignment.ALIGNLEFTUP);
                    usingRightArm = (alignment == 0) ? true : false;
                    distance = Math.abs(distance);

                    resetEncoders();
                    auto = AutoEnum.COLLECTPROCESSFORPLACING;
                    break;
                case COLLECTPROCESSFORPLACING:
                    if (BOX[0][columnNumber] == NONE) {

                    } else if (BOX[1][columnNumber] == NONE) {
                        lift = 1;
                    } else {
                        lift = 2;
                    }

                    robotHandler.drive.mecanum.setMecanum(Math.toRadians(direction), 0.7, PIDrotationOut, 1.0);
                    if (traveledEncoderTicks(Constants.drive.countsPerInches(distance))) {// || wait.milliseconds() > 5000) {
                        robotHandler.drive.stop();
                        wait.reset();
                        auto = AutoEnum.ALIGNDRIVEINTOCRYPTO;
                    }

                    break;


            }

            if (lift == 0) {
                if (Math.abs(motorLift.getEncoderPosition()) < 30) {
                    motorLift.setPower(0);
                } else {
                    motorLift.setPower(1);
                }
            } else if (lift == 1) {
                if (Math.abs(motorLift.getEncoderPosition()) > 1500) {
                    motorLift.setPower(0);
                } else {
                    motorLift.setPower(-1);
                }
            } else if (lift == 2) {
                if (Math.abs(motorLift.getEncoderPosition()) > 3000) {
                    motorLift.setPower(0);
                } else {
                    motorLift.setPower(-1);
                }
            }

            telemetry.addData("Glyph", getGlyph());
            telemetry.addData("Target Column", targetColumn);
            telemetry.addData("Front Glyph", frontGlyph);
            telemetry.addData("Back Glyph", backGlyph);
            telemetry.addData("PID ROTATION", PIDrotationOut);
            telemetry.addData("distance", distance);
            telemetry.addData("column number", columnNumber);
            telemetry.addData("prevous column", previousColumn);
            printBox();
            telemetry.update();
            prervPos = motorCollectLeft.getEncoderPosition();
            prervTime = collectorRPMTimer.milliseconds();
        }

    }

    //JEWEL FUNCTIONS HERE
    public void jewelCSLEDON() {
        csJewelRight.getValue(PineappleEnum.PineappleSensorEnum.CSLEDON);
        csJewelLeft.getValue(PineappleEnum.PineappleSensorEnum.CSLEDON);

    }

    public void jewelCSLEDOFF() {
        csJewelRight.getValue(PineappleEnum.PineappleSensorEnum.CSLEDOFF);
        csJewelLeft.getValue(PineappleEnum.PineappleSensorEnum.CSLEDOFF);

    }

    public int jewelDown() {
        servoJewelHit.setPosition(Constants.auto.jewel.JEWELHITCENTER);

        servoJewel.setPosition(Constants.auto.jewel.JEWELDOWN);
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

    public Constants.auto.jewel.jewelHitSide jewelHitSideSimple() {
        Constants.auto.jewel.jewelState left = getLeftCSJewelState();
        Constants.auto.jewel.jewelState right = getRightCSJewelState();
        Constants.auto.jewel.jewelState state;
        state = left;
//        if (left == right) {
//            state = left;
//        } else if (left == NON_NON && right != NON_NON) {
//            state = right;
//        } else if (right == NON_NON && left != NON_NON) {
//            state = left;
//        } else {
//            state = NON_NON;
//        }
        return (switchColor == RelicRecoveryEnums.AutoColor.RED) ? (state == RED_BLUE) ? RIGHT : LEFT : (state == RED_BLUE) ? LEFT : RIGHT;

    }

    public Constants.auto.jewel.jewelHitSide jewelHitSide() {
        Constants.auto.jewel.jewelState left = getLeftCSJewelState();
        Constants.auto.jewel.jewelState right = getRightCSJewelState();
        Constants.auto.jewel.jewelState state;
        if (left == right) {
            if (left != NON_NON) {
                state = left;
            } else {
                state = jewelState;
            }

        } else if (left != NON_NON && left == jewelState) {
            state = left;
        } else if (right != NON_NON && right == jewelState) {
            state = right;
        } else if (left == NON_NON && jewelState == NON_NON && right != NON_NON) {
            state = right;
        } else if (right == NON_NON && jewelState == NON_NON && left != NON_NON) {
            state = left;
        } else {
            state = NON_NON;
        }
        if (state == NON_NON) {
            return Constants.auto.jewel.jewelHitSide.NONE;
        }
        return (switchColor == RelicRecoveryEnums.AutoColor.RED) ? (state == RED_BLUE) ? RIGHT : LEFT : (state == RED_BLUE) ? LEFT : RIGHT;

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

    public static Constants.auto.jewel.jewelState getJewelConfig(Image img, VuforiaTrackableDefaultListener track, CameraCalibration camCal, Telemetry telemetry) {
        try {
            OpenGLMatrix pose = track.getRawPose();
            if (pose != null && img != null && img.getPixels() != null) {
                Matrix34F rawPose = new Matrix34F();
                float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);
                rawPose.setData(poseData);
                float[][] corners = new float[4][2];
                corners[0] = Tool.projectPoint(camCal, rawPose, new Vec3F(120, -55, 100)).getData();//UL TODO FIND NEW LOCATIONS
                corners[1] = Tool.projectPoint(camCal, rawPose, new Vec3F(340, -55, 100)).getData();//UR TODO FIND NEW LOCATIONS
                corners[2] = Tool.projectPoint(camCal, rawPose, new Vec3F(340, -300, 100)).getData();//LR TODO FIND NEW LOCATIONS
                corners[3] = Tool.projectPoint(camCal, rawPose, new Vec3F(120, -300, 100)).getData();//LL TODO FIND NEW LOCATIONS
                Bitmap bm = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
                ByteBuffer pix = img.getPixels();
                bm.copyPixelsFromBuffer(pix);
                SaveImage(bm, "original");
                Mat crop = new Mat(bm.getHeight(), bm.getWidth(), CvType.CV_8UC3);
                Utils.bitmapToMat(bm, crop);
                float x = Math.min(Math.min(corners[1][0], corners[3][0]), Math.min(corners[0][0], corners[2][0]));
                float y = Math.min(Math.min(corners[1][1], corners[3][1]), Math.min(corners[0][1], corners[2][1]));
                float width = Math.max(Math.abs(corners[0][0] - corners[2][0]), Math.abs(corners[1][0] - corners[3][0]));
                float height = Math.max(Math.abs(corners[0][1] - corners[2][1]), Math.abs(corners[1][1] - corners[3][1]));
                x = Math.max(x, 0);
                y = Math.max(y, 0);
                if (width < 20 || height < 20) {
                    return NON_NON;
                }
                width = (x + width > crop.cols()) ? crop.cols() - x : width;
                height = (x + height > crop.rows()) ? crop.rows() - x : height;
                Mat cropped = new Mat(crop, new Rect((int) x, (int) y, (int) width, (int) height));
                SaveImage(matToBitmap(cropped), "crop");
                Imgproc.cvtColor(cropped, cropped, Imgproc.COLOR_RGB2HSV_FULL);
                Mat mask = new Mat();
                Core.inRange(cropped, new Scalar(50, 20, 70), new Scalar(255, 255, 120), mask);
                SaveImage(matToBitmap(mask), "mask");
                Moments mmnts = Imgproc.moments(mask, true);
                if ((mmnts.get_m10() / mmnts.get_m00()) < cropped.cols() / 2) {
                    return BLUE_RED;
                } else {
                    return RED_BLUE;
                }

            }
            return NON_NON;
        } catch (Exception e)

        {
            return NON_NON;
        }
    }

    //ALIGN FUNCTIONS HERE

    double[] encoderReset = {0.0, 0.0, 0.0, 0.0};

    public void resetEncoders() {
        encoderReset = getEncoderPositions();
    }

    public double[] getEncoderPositions() {
        double[] encoders = {driveFrontLeft.getEncoderPosition(), driveFrontRight.getEncoderPosition(), driveBackLeft.getEncoderPosition(), driveBackRight.getEncoderPosition()};
        return encoders;
    }

    public boolean traveledEncoderTicks(int ticks) {
        return (getTraveledEncoderTicks() > ticks);
    }

    public int getTraveledEncoderTicks() {
        double[] currentPosition = getEncoderPositions();
        double total = 0.0;
        for (int i = 0; i < 4; i++) {
            currentPosition[i] = Math.abs(currentPosition[i] - encoderReset[i]);
            total += currentPosition[i];
        }
        double average = total / 4;
        double[] distance = {0.0, 0.0, 0.0, 0.0};
        double greatestOffset = 0.0;
        int greatestOffsetMotor = 0;
        for (int i = 0; i < 4; i++) {
            distance[i] = currentPosition[i] - average;
            if (distance[i] > greatestOffset) {
                greatestOffset = distance[i];
                greatestOffsetMotor = i;
            }
        }
        total = 0;
        for (int i = 0; i < 4; i++) {
            if (greatestOffsetMotor != i) {
                total += currentPosition[i];
            }
        }
        average = total / 3;
        return (int) average;
    }

    //GLYPH FUNCTIONS HERE
    public int columnNumber(RelicRecoveryVuMark vuMark) {
        return (vuMark == RelicRecoveryVuMark.LEFT) ? 0 : (vuMark == RelicRecoveryVuMark.CENTER) ? 1 : 2;
    }


    public static Constants.auto.autoGlyph.glyph[][] moveGlyphs(Constants.auto.autoGlyph.glyph[][] box) {
        Constants.auto.autoGlyph.glyph[][] newBox = {
                {NONE, NONE, NONE},
                {NONE, NONE, NONE},
                {NONE, NONE, NONE},
                {NONE, NONE, NONE}
        };
        for (int i = 0; i < newBox.length; i++) {
            for (int j = 0; j < newBox[i].length; j++) {
                newBox[i][j] = box[i][j];
            }
        }
        return newBox;
    }

    public RelicRecoveryVuMark getColumn(glyph firstGlyph, glyph secondGlyph) {
        int[] cipherPoints = new int[3];

        for (int i = 0; i < 3; i++) {
            if (canGlyphsGoInColumn(i, firstGlyph, secondGlyph)) {
                glyph[][] potBox = addGlyphsToColumnAlg(i, moveGlyphs(BOX), firstGlyph, secondGlyph);
                boolean[] workingCipher = {true, true, true, true, true, true};
                for (int k = 0; k < 6; k++) {
                    for (int j = 0; j < 4; j++) {
                        for (int l = 0; l < 3; l++) {
                            if (potBox[j][l] != NONE && potBox[j][l] != Constants.auto.autoGlyph.CIPHERS[k][j][l]) {
                                j = 5;
                                l = 4;
                                workingCipher[k] = false;
                            }
                        }
                    }
                }
                cipherPoints[i] = cipherPointsFinder(workingCipher);
            }
        }
        int max = getMax(cipherPoints);
        if (max == 1 && cipherPoints[0] == max) {
            max = 0;
        }
        if (max == 1 && cipherPoints[2] == max) {
            max = 2;
        }
        switch (max) {
            case 0:
                return RelicRecoveryVuMark.LEFT;
            case 1:
                return RelicRecoveryVuMark.CENTER;
            case 2:
                return RelicRecoveryVuMark.RIGHT;
            default:
                return RelicRecoveryVuMark.LEFT;

        }
    }

    public static int getMax(int[] inputArray) {
        int maxValue = inputArray[0];
        int maxPos = 0;
        for (int i = 1; i < inputArray.length; i++) {
            if (inputArray[i] > maxValue) {
                maxValue = inputArray[i];
                maxPos = i;
            }
        }
        return maxPos;
    }

    private glyph[][] addGlyphsToColumnAlg(int column, glyph[][] box, glyph firstGlyph, glyph secondGlyph) {
        for (int i = 0; i < 4; i++) {
            if (box[i][column] == NONE) {
                if (secondGlyph != NONE) {
                    box[i][column] = secondGlyph;
                    box[i + 1][column] = firstGlyph;
                } else {
                    box[i][column] = firstGlyph;
                }
                i = 5;
            }

        }
        return box;
    }

    public void addGlyphsToColumn(RelicRecoveryVuMark column, glyph firstGlyph, glyph secondGlyph) {
        int columnNumb = 1;
        switch (column) {
            case LEFT:
                columnNumb = 0;
                break;
            case CENTER:
                columnNumb = 1;
                break;
            case RIGHT:
                columnNumb = 2;
                break;
        }
        for (int i = 0; i < 4; i++) {
            if (BOX[i][columnNumb] == NONE) {
                if (secondGlyph != NONE) {
                    BOX[i][columnNumb] = secondGlyph;
                    BOX[i + 1][columnNumb] = firstGlyph;
                } else {
                    BOX[i][columnNumb] = firstGlyph;
                }
                i = 5;
            }

        }
    }

    private boolean canGlyphsGoInColumn(int column, glyph firstGlyph, glyph secondGlyph) {
        int numbOfGlyphs = 0;
        if (firstGlyph != NONE) {
            numbOfGlyphs++;
        }
        if (secondGlyph != NONE) {
            numbOfGlyphs++;
        }
        int numbOfAvalibaleSpot = 0;
        for (int i = 0; i < 4; i++) {
            if (BOX[i][column] == NONE) {
                numbOfAvalibaleSpot++;
            }
        }
        return numbOfAvalibaleSpot >= numbOfGlyphs;
    }

    private static int cipherPointsFinder(boolean[] cipher) {
        int val = 0;
        int points = 0;
        for (boolean cipherBool : cipher) {
            if (cipherBool) {
                points += (val == 0 || val == 1) ? 4 : (val == 2 || val == 3) ? 2 : 1;
            }
            val++;
        }
        return points;
    }


    //COLLECT FUNCTIONS HERE

    public Constants.auto.autoGlyph.glyph getGlyph() {
        double average = (glyphColor.red() + glyphColor.green() + glyphColor.blue()) / 3.0;
        double averageWithoutRed = (glyphColor.green() + glyphColor.blue()) / 2.0;
        if (average > 0) {
            if (averageWithoutRed <= 3.5) {
                return BROWN;
            } else {
                return GREY;
            }
        } else {
            return NONE;
        }
    }

    //GENERAL FUNCTIONS HERE
    public void printBox() {
        for (int i = 3; i >= 0; i--) {
            telemetry.addLine(getGlyphSymbol(BOX[i][0]) + getGlyphSymbol(BOX[i][1]) + getGlyphSymbol(BOX[i][2]));
        }
    }

    public String getGlyphSymbol(glyph gly) {
        if (gly == NONE) {
            return "x";
        } else if (gly == BROWN) {
            return FontFormating.emptyBox;
        } else {
            return FontFormating.filledBox;
        }
    }
}
