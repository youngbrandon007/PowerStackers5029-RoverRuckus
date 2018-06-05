package org.firstinspires.ftc.teamcode.RelicRecoveryWorlds;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/**
 * Created by Brandon on 1/8/2018.
 */
@Autonomous(name = "PIDTUNE")
public class WorldPIDTuning extends WorldConfig {
    double PIDrotationOut = 0;

    @Override
    public void init() {
        config(this);
    }

    @Override
    public void loop() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double gyro = angles.firstAngle;


        PIDrotationOut = -PID.getOutput(gyro, -90);//gyro
        telemetry.addData("Output", PIDrotationOut);
        telemetry.addData("Angle", gyro);
//        robotHandler.drive.mecanum.setMecanumThridPerson(Math.toRadians(WorldConstants.auto.aligning.AlignDriveOffPlatformDirection[colorPositionInt]), 1, PIDrotationOut, 1, Math.toRadians(gyro));

        robotHandler.drive.tank.setPower(PIDrotationOut, PIDrotationOut);
        telemetry.update();
    }

//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        config(this);
//
//        waitForStart();
//        AnalogInput PPot = hardwareMap.analogInput.get("P");
//        AnalogInput IPot = hardwareMap.analogInput.get("I");
//        AnalogInput DPot = hardwareMap.analogInput.get("D");
//
//        DecimalFormat decimalFormat = new DecimalFormat("#.########");
//        double P = Double.parseDouble(decimalFormat.format(PPot.getVoltage() / 20));
//        double I = Double.parseDouble(decimalFormat.format(IPot.getVoltage() / 50));
//
//        double D = Double.parseDouble(decimalFormat.format(DPot.getVoltage() / 50));
//        while (!opModeIsActive()) {
//            telemetry.addData("P:", P);
//            telemetry.addData("I:", I);
//            telemetry.addData("D:", D);
//            PPot.getVoltage();
//            P = Double.parseDouble(decimalFormat.format(PPot.getVoltage() / 20));
//            I = Double.parseDouble(decimalFormat.format(IPot.getVoltage() / 100));
//            D = Double.parseDouble(decimalFormat.format(DPot.getVoltage() / 20));
//            telemetry.update();
//        }
//        telemetry.addData("PID", "Tuning Finished Press play");
//        telemetry.update();
//        waitForStart();
//        robotHandler.auto.gyroTurnPID(90, P, I, D, navx_device);
//
//    }

}



