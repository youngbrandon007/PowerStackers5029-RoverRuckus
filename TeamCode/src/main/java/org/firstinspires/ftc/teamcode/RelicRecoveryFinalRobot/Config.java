package org.firstinspires.ftc.teamcode.RelicRecoveryFinalRobot;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogOutput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.Old_Robots.RelicRecovery.RelicRecoveryOfficalFile.RelicResources.RelicRecoveryConstants;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleConfigLinearOpMode;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleEnum;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleMotor;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleRobot;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleSensor;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleServo;
import org.firstinspires.ftc.teamcode.Old_Robots.RelicRecovery.RelicRecoveryOfficalFile.RelicResources.RelicRecoveryEnums;

import static org.firstinspires.ftc.teamcode.RelicRecoveryFinalRobot.Constants.auto.autoGlyph.glyph.BROWN;
import static org.firstinspires.ftc.teamcode.RelicRecoveryFinalRobot.Constants.auto.autoGlyph.glyph.GREY;
import static org.firstinspires.ftc.teamcode.RelicRecoveryFinalRobot.Constants.auto.autoGlyph.glyph.NONE;

/**
 * Created by Brandon on 1/8/2018.
 */

public abstract class Config extends PineappleConfigLinearOpMode {

    //DRIVE MOTORS
    public PineappleMotor driveFrontRight;
    public PineappleMotor driveFrontLeft;
    public PineappleMotor driveBackRight;
    public PineappleMotor driveBackLeft;

    //MOTORS
    public PineappleMotor motorLift;
    public PineappleMotor motorCollectRight;
    public PineappleMotor motorCollectLeft;
    public PineappleMotor motorRelic;

    //SERVOS
    public PineappleServo servoFlipR;
    public PineappleServo servoFlipL;
    public PineappleServo servoAlignLeft;
    public PineappleServo servoAlignRight;
    public PineappleServo servoJewelHit;
    public PineappleServo servoJewel;
    public PineappleServo servoRelicGrab;
    public PineappleServo servoRelicTurn;
    public PineappleServo servoGlyphStop;
    //JEWEL
    public Constants.auto.jewel.jewelState jewelState = Constants.auto.jewel.jewelState.NON_NON;
    //GLYPH
    Constants.auto.autoGlyph.glyph[][] BOX = {
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
    public PineappleSensor csJewelLeft;
    public PineappleSensor csJewelRight;
    public OpticalDistanceSensor opticalGlyph;
    public ColorSensor glyphColor;

    //GYRO
    public final int NAVX_DIM_I2C_PORT = 0;
    public AHRS navx_device;
    public navXPIDController yawPIDController;
    public final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;
    public final double TOLERANCE_DEGREES = 2.0;
    public final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
    public final double MAX_MOTOR_OUTPUT_VALUE = 1.0;
    public final double YAW_PID_P = Constants.PID.P;
    public final double YAW_PID_I = Constants.PID.I;
    public final double YAW_PID_D = Constants.PID.D;
    public navXPIDController.PIDResult yawPIDResult;

    public boolean calibration_complete = false;


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



    @Override
    public void config(LinearOpMode linearOpMode) {
        robotHandler = new PineappleRobot(linearOpMode);

        //DRIVE MOTORS
        driveFrontRight = robotHandler.motorHandler.newDriveMotor("FR", 1, false, false, PineappleEnum.MotorLoc.RIGHTFRONT, PineappleEnum.MotorType.NEV40);
        driveFrontLeft = robotHandler.motorHandler.newDriveMotor("FL", 1, false, false, PineappleEnum.MotorLoc.LEFTFRONT, PineappleEnum.MotorType.NEV40);
        driveBackRight = robotHandler.motorHandler.newDriveMotor("BR", 1, false, false, PineappleEnum.MotorLoc.RIGHTBACK, PineappleEnum.MotorType.NEV40);
        driveBackLeft = robotHandler.motorHandler.newDriveMotor("BL", 1, false, false, PineappleEnum.MotorLoc.LEFTBACK, PineappleEnum.MotorType.NEV40);

        //MOTORS
        motorLift = robotHandler.motorHandler.newMotor("ML");
        motorCollectRight = robotHandler.motorHandler.newMotor("MCR");
        motorCollectLeft = robotHandler.motorHandler.newMotor("MCL");
        motorRelic = robotHandler.motorHandler.newMotor("MR");

        //SERVOS
        servoFlipL = robotHandler.servoHandler.newLimitServo("SL", 202.5, Constants.flip.leftDown);
        servoFlipR = robotHandler.servoHandler.newLimitServo("SR", 202.5, Constants.flip.rightDown);
        servoAlignLeft = robotHandler.servoHandler.newLimitServo("SAL", 202.5, Constants.alignment.ALIGNLEFTINIT);
        servoAlignRight = robotHandler.servoHandler.newLimitServo("SAR", 202.5, Constants.alignment.ALIGNRIGHTINIT);
        servoJewel = robotHandler.servoHandler.newLimitServo("SJ", 202.5, Constants.auto.jewel.JEWELUP);
        servoJewelHit = robotHandler.servoHandler.newLimitServo("SJH", 202.5, Constants.auto.jewel.JEWELHITLEFT);
        servoRelicTurn = robotHandler.servoHandler.newLimitServo("SRT", 202.5, Constants.relic.turnFold);
        servoRelicGrab = robotHandler.servoHandler.newLimitServo("SRG", 202.5, Constants.relic.grabIn);
        servoGlyphStop = robotHandler.servoHandler.newLimitServo("SGS", 202.5, Constants.flip.stopDown);

        //SENSORS
        csJewelLeft = robotHandler.sensorHandler.newColorSensor("CSJL");
        csJewelRight = robotHandler.sensorHandler.newColorSensor("CSJR");

        navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);
        yawPIDController = new navXPIDController(navx_device,
                navXPIDController.navXTimestampedDataSource.YAW);
        yawPIDController.setSetpoint(0);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
        yawPIDController.enable(true);

        limitLeftBack = linearOpMode.hardwareMap.digitalChannel.get("LLB");
        limitLeftSide = linearOpMode.hardwareMap.digitalChannel.get("LLS");
        limitRightBack = linearOpMode.hardwareMap.digitalChannel.get("LRB");
        limitRightSide = linearOpMode.hardwareMap.digitalChannel.get("LRS");

        opticalGlyph = linearOpMode.hardwareMap.opticalDistanceSensor.get("OPT");
        glyphColor = linearOpMode.hardwareMap.colorSensor.get("GC");
    }

    public void loadSwitchBoard() {
       switchColor = (robotHandler.switchBoard.loadDigital("color")) ? RelicRecoveryEnums.AutoColor.RED : RelicRecoveryEnums.AutoColor.BLUE;
        switchPosition = (robotHandler.switchBoard.loadDigital("position")) ? RelicRecoveryEnums.StartingPosition.FRONT : RelicRecoveryEnums.StartingPosition.BACK;
        switchColorPosition = (switchColor == RelicRecoveryEnums.AutoColor.RED) ? (switchPosition == RelicRecoveryEnums.StartingPosition.FRONT) ? RelicRecoveryEnums.ColorPosition.REDFRONT : RelicRecoveryEnums.ColorPosition.REDBACK : (switchPosition == RelicRecoveryEnums.StartingPosition.FRONT) ? RelicRecoveryEnums.ColorPosition.BLUEFRONT : RelicRecoveryEnums.ColorPosition.BLUEBACK;
        switchDelayEnabled = robotHandler.switchBoard.loadDigital("delayEnabled");
        slideDelay = (switchDelayEnabled) ? Math.round((1-robotHandler.switchBoard.loadAnalog("delay"))*30)/2 : 0.0;

        switchMoreGlyphs = robotHandler.switchBoard.loadDigital("moreGlyph");
        switchPID = robotHandler.switchBoard.loadDigital("PID");
        switchJewels = robotHandler.switchBoard.loadDigital("jewel");
        switchHitBadJewel = robotHandler.switchBoard.loadDigital("badJewel");

        colorPositionInt = (switchColor == RelicRecoveryEnums.AutoColor.RED) ? (switchPosition == RelicRecoveryEnums.StartingPosition.FRONT) ? 0 : 2 : (switchPosition == RelicRecoveryEnums.StartingPosition.FRONT) ? 1 : 3;
    }

    public void displaySwitchBorad(boolean switchGlyphWhite) {
        String autonomousDescription = switchColor + " " + switchPosition + "     " + slideDelay + "-SECONDS-" + FontFormating.getMark(switchDelayEnabled) + "     PID-" + FontFormating.getMark(switchPID);
        String autonomousSettings = "JEWELS-" + FontFormating.getMark(switchJewels) + "     BAD-" + FontFormating.getMark(switchHitBadJewel) + "     MORE " + FontFormating.getBox(switchGlyphWhite) + "-" + FontFormating.getMark(switchMoreGlyphs);
        telemetry.addLine(autonomousDescription);
        telemetry.addLine(autonomousSettings);
    }
}
