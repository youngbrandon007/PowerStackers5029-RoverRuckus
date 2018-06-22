package org.firstinspires.ftc.teamcode.RelicRecoveryWorlds;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.FontFormating;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PSConfigOpMode;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PSEnum;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PSMotor;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PSRobot;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PSSensor;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PSServo;

import static org.firstinspires.ftc.teamcode.RelicRecoveryWorlds.WorldConstants.auto.autoGlyph.glyph.NONE;

/**
 * Created by Brandon on 1/8/2018.
 */

public abstract class WorldConfig extends PSConfigOpMode {

    //DRIVE MOTORS
    public PSMotor driveFrontRight;
    public PSMotor driveFrontLeft;
    public PSMotor driveBackRight;
    public PSMotor driveBackLeft;

    //MOTORS
    public PSMotor motorLift;
    public PSMotor motorCollectRight;
    public PSMotor motorCollectLeft;
    public PSMotor motorRelic;

    //SERVOS
    public PSServo servoFlipR;
    public PSServo servoFlipL;
    public PSServo servoAlignLeft;
    public PSServo servoAlignRight;
    public PSServo servoJewelHit;
    public PSServo servoJewel;
    public PSServo servoRelicGrab;
    public PSServo servoRelicTurn;
    public PSServo servoGlyphStop;
    //JEWEL
    public WorldConstants.auto.jewel.jewelState jewelState = WorldConstants.auto.jewel.jewelState.NON_NON;
    //GLYPH
    WorldConstants.auto.autoGlyph.glyph[][] BOX = {
            {NONE, NONE, NONE},
            {NONE, NONE, NONE},
            {NONE, NONE, NONE},
            {NONE, NONE, NONE}
    };
    //SENSORS
    public DigitalChannel limitLeftBack;
    public DigitalChannel limitLeftSide;
    public DigitalChannel limitRightBack;
    public DigitalChannel limitRightSide;
    public PSSensor csJewelLeft;
    public PSSensor csJewelRight;
    public OpticalDistanceSensor opticalGlyph;
    public OpticalDistanceSensor opticalRight;
    public OpticalDistanceSensor opticalLeft;
    //    public ColorSensor glyphColor;
    public ColorSensor backGlyphColor;
    public DistanceSensor glyphDist;
    public ColorSensor collectColor;
    public DistanceSensor collectDist;
    public WorldPID PIDT;

    //GYRO

    public WorldPID PID = new WorldPID(WorldConstants.PID.P, WorldConstants.PID.I, WorldConstants.PID.D);


    //SWITCH BOARD
    public RelicRecoveryEnums.AutoColor switchColor = RelicRecoveryEnums.AutoColor.RED;
    public RelicRecoveryEnums.StartingPosition switchPosition = RelicRecoveryEnums.StartingPosition.FRONT;
    public RelicRecoveryEnums.ColorPosition switchColorPosition = RelicRecoveryEnums.ColorPosition.REDFRONT;
    public int colorPositionInt = 0;
    public boolean switchDelayEnabled = false;
    public double slideDelay = 0.0;
    //Still need detecting
    public boolean switchJewels = true;
    public boolean switchMoreGlyphs = true;
    public boolean switchPID = true;
    public boolean switchHitBadJewel = false;
    public boolean firstReset = true;
    public boolean otherfirstReset = true;
    public int numbCol = 0;
    public boolean firstGlyph = true;
    BNO055IMU imu;
    // State used for updating telemetry
    Orientation angles;

    @Override
    public void config(OpMode opmode) {
        robotHandler = new PSRobot(opmode);

        //DRIVE MOTORS
        driveFrontRight = robotHandler.motorHandler.newDriveMotor("FR", 1, false, false, PSEnum.MotorLoc.RIGHTFRONT, PSEnum.MotorType.NEV40);
        driveFrontLeft = robotHandler.motorHandler.newDriveMotor("FL", 1, false, false, PSEnum.MotorLoc.LEFTFRONT, PSEnum.MotorType.NEV40);
        driveBackRight = robotHandler.motorHandler.newDriveMotor("BR", 1, false, false, PSEnum.MotorLoc.RIGHTBACK, PSEnum.MotorType.NEV40);
        driveBackLeft = robotHandler.motorHandler.newDriveMotor("BL", 1, false, false, PSEnum.MotorLoc.LEFTBACK, PSEnum.MotorType.NEV40);

        //MOTORS
        motorLift = robotHandler.motorHandler.newMotor("ML");
        motorCollectRight = robotHandler.motorHandler.newMotor("MCR");
        motorCollectLeft = robotHandler.motorHandler.newMotor("MCL");
        motorRelic = robotHandler.motorHandler.newMotor("MR");

        //SERVOS
        servoFlipL = robotHandler.servoHandler.newLimitServo("SL", 202.5, WorldConstants.flip.leftDown);
        servoFlipR = robotHandler.servoHandler.newLimitServo("SR", 202.5, WorldConstants.flip.rightDown);
        servoAlignLeft = robotHandler.servoHandler.newLimitServo("SAL", 202.5, WorldConstants.alignment.ALIGNLEFTINIT);
        servoAlignRight = robotHandler.servoHandler.newLimitServo("SAR", 202.5, WorldConstants.alignment.ALIGNRIGHTINIT);
        servoJewel = robotHandler.servoHandler.newLimitServo("SJ", 202.5, WorldConstants.auto.jewel.JEWELUP);
        servoJewelHit = robotHandler.servoHandler.newLimitServo("SJH", 202.5, WorldConstants.auto.jewel.JEWELHITLEFT);
        servoRelicTurn = robotHandler.servoHandler.newLimitServo("SRT", 202.5, WorldConstants.relic.turnFold);
        servoRelicGrab = robotHandler.servoHandler.newLimitServo("SRG", 202.5, WorldConstants.relic.grabIn);
        servoGlyphStop = robotHandler.servoHandler.newLimitServo("SGS", 202.5, WorldConstants.flip.stopDown);

        //SENSORS
        csJewelLeft = robotHandler.sensorHandler.newColorSensor("CSJL");
        csJewelRight = robotHandler.sensorHandler.newColorSensor("CSJR");

        limitLeftBack = hardwareMap.digitalChannel.get("LLB");
        limitLeftSide = hardwareMap.digitalChannel.get("LLS");
        limitRightBack = hardwareMap.digitalChannel.get("LRB");
        limitRightSide = hardwareMap.digitalChannel.get("LRS");

        opticalGlyph = hardwareMap.opticalDistanceSensor.get("OPT");
        opticalLeft = hardwareMap.opticalDistanceSensor.get("GODSL");
        opticalRight = hardwareMap.opticalDistanceSensor.get("GODSR");
//        glyphColor = hardwareMap.colorSensor.get("GC");
        backGlyphColor = hardwareMap.get(ColorSensor.class, "GCD");
        glyphDist = hardwareMap.get(DistanceSensor.class, "GCD");
        collectColor = hardwareMap.get(ColorSensor.class, "CCD");
        collectDist = hardwareMap.get(DistanceSensor.class, "CCD");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        //parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        PID.setOutputLimits(-1, 1);
    }

    public void loadSwitchBoard() {
       switchColor = (robotHandler.switchBoard.loadDigital("color")) ? RelicRecoveryEnums.AutoColor.RED : RelicRecoveryEnums.AutoColor.BLUE;
//        switchPosition = (robotHandler.switchBoard.loadDigital("position")) ? RelicRecoveryEnums.StartingPosition.FRONT : RelicRecoveryEnums.StartingPosition.BACK;
        switchColorPosition = (switchColor == RelicRecoveryEnums.AutoColor.RED) ? (switchPosition == RelicRecoveryEnums.StartingPosition.FRONT) ? RelicRecoveryEnums.ColorPosition.REDFRONT : RelicRecoveryEnums.ColorPosition.REDBACK : (switchPosition == RelicRecoveryEnums.StartingPosition.FRONT) ? RelicRecoveryEnums.ColorPosition.BLUEFRONT : RelicRecoveryEnums.ColorPosition.BLUEBACK;
//        switchDelayEnabled = robotHandler.switchBoard.loadDigital("delayEnabled");
//        slideDelay = (switchDelayEnabled) ? Math.round((1-robotHandler.switchBoard.loadAnalog("delay"))*30)/2 : 0.0;
//
//        switchMoreGlyphs = robotHandler.switchBoard.loadDigital("moreGlyph");
//        switchPID = robotHandler.switchBoard.loadDigital("PID");
        switchJewels = robotHandler.switchBoard.loadDigital("jewel");
//        switchHitBadJewel = robotHandler.switchBoard.loadDigital("badJewel");

        colorPositionInt = (switchColor == RelicRecoveryEnums.AutoColor.RED) ? (switchPosition == RelicRecoveryEnums.StartingPosition.FRONT) ? 0 : 2 : (switchPosition == RelicRecoveryEnums.StartingPosition.FRONT) ? 1 : 3;
    }

    public void displaySwitchBorad(boolean switchGlyphWhite) {
        String autonomousDescription = switchColor + " " + switchPosition + "     " + slideDelay + "-SECONDS-" + FontFormating.getMark(switchDelayEnabled) + "     PID-" + FontFormating.getMark(switchPID);
        String autonomousSettings = "JEWELS-" + FontFormating.getMark(switchJewels) + "     BAD-" + FontFormating.getMark(switchHitBadJewel) + "     MORE " + FontFormating.getBox(switchGlyphWhite) + "-" + FontFormating.getMark(switchMoreGlyphs);
        telemetry.addLine(autonomousDescription);
        telemetry.addLine(autonomousSettings);
    }
}
