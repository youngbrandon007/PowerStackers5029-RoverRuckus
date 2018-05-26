package org.firstinspires.ftc.teamcode.RelicRecoveryFinalRobot;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.AnalogInput;

import java.io.Console;
import java.text.DecimalFormat;

/**
 * Created by Brandon on 1/8/2018.
 */
@Autonomous(name = "PIDTUNE")
public class PIDTuning extends Config {

    @Override
    public void runOpMode() throws InterruptedException {
        config(this);

        waitForStart();
        AnalogInput PPot = hardwareMap.analogInput.get("P");
        AnalogInput IPot = hardwareMap.analogInput.get("I");
        AnalogInput DPot = hardwareMap.analogInput.get("D");

        DecimalFormat decimalFormat = new DecimalFormat("#.########");
        double P = Double.parseDouble(decimalFormat.format(PPot.getVoltage() / 20));
        double I = Double.parseDouble(decimalFormat.format(IPot.getVoltage() / 50));

        double D = Double.parseDouble(decimalFormat.format(DPot.getVoltage() / 50));
        while (!opModeIsActive()) {
            telemetry.addData("P:", P);
            telemetry.addData("I:", I);
            telemetry.addData("D:", D);
            PPot.getVoltage();
            P = Double.parseDouble(decimalFormat.format(PPot.getVoltage() / 20));
            I = Double.parseDouble(decimalFormat.format(IPot.getVoltage() / 100));
            D = Double.parseDouble(decimalFormat.format(DPot.getVoltage() / 20));
            telemetry.update();
        }
        telemetry.addData("PID", "Tuning Finished Press play");
        telemetry.update();
        waitForStart();
        robotHandler.auto.gyroTurnPID(90, P, I, D, navx_device);

    }

}



