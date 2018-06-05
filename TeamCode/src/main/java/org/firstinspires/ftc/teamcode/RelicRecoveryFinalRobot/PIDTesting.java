package org.firstinspires.ftc.teamcode.RelicRecoveryFinalRobot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.AnalogInput;

import java.text.DecimalFormat;

/**
 * Created by ftcpi on 2/7/2018.
 */
@Autonomous(name = "PID")
@Disabled
public class PIDTesting extends Config{
    @Override
    public void runOpMode() throws InterruptedException {

        DecimalFormat decimalFormat = new DecimalFormat("#.#####");
        config(this);
        while (!calibration_complete) {
            calibration_complete = !navx_device.isCalibrating();

        }
        navx_device.zeroYaw();
        AnalogInput PPot = hardwareMap.analogInput.get("P");
        AnalogInput IPot = hardwareMap.analogInput.get("I");
        AnalogInput DPot = hardwareMap.analogInput.get("D");

        double P = Double.parseDouble(decimalFormat.format(PPot.getVoltage()/50));
        double I = Double.parseDouble(decimalFormat.format(IPot.getVoltage()/100));
        double D = Double.parseDouble(decimalFormat.format(DPot.getVoltage()/50));
        while (!opModeIsActive()) {
            telemetry.addData("P:", P);
            telemetry.addData("I*1000:", I*1000);
            telemetry.addData("D:", D);
            PPot.getVoltage();
//            P = Double.parseDouble(decimalFormat.format(PPot.getVoltage()/50));
            P = Constants.PID.P;
//            I = Double.parseDouble(decimalFormat.format(IPot.getVoltage()/100));
//            D = Double.parseDouble(decimalFormat.format(DPot.getVoltage()/50));
            I = Constants.PID.I;
            D = Constants.PID.D;
            telemetry.update();
        }
        telemetry.addData("PID", "Tuning Finished Press play");
        telemetry.update();
        waitForStart();
        //robotHandler.auto.gyroTurnPID(90, P, I, D, navx_device);

    }
}
