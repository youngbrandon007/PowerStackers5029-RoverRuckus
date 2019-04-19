package org.firstinspires.ftc.teamcode.roverRuckus.Testing;

import android.graphics.Color;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cConfigureChannelCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.hardware.LEDRiver;

@TeleOp(name = "LEDRiver")
public class LEDRiverTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        LynxModule revHub = hardwareMap.get(LynxModule.class, "Rev Expansion Hub 2");
        try {
            new LynxI2cConfigureChannelCommand(revHub, 1, LynxI2cConfigureChannelCommand.SpeedCode.FAST_400K).send();
        } catch (LynxNackException | InterruptedException ex) {
            ex.printStackTrace();
        }

        LEDRiver ledRiver = hardwareMap.get(LEDRiver.IMPL, "LEDriver");
        ledRiver.setLEDCount(110);
        ledRiver.save();
        ledRiver.setMode(LEDRiver.Mode.SOLID);
        ledRiver.setLEDMode(LEDRiver.LEDMode.RGB);
        ledRiver.setColorDepth(LEDRiver.ColorDepth.BIT_24);
        ledRiver.setColor(0, new LEDRiver.Color(255, 0, 0, 0));
        ledRiver.setColor(1, Color.GREEN);
        ledRiver.setColor(2, Color.BLACK);


        waitForStart();

        ledRiver.apply();
        Thread.sleep(1000);

        ledRiver.setColor(Color.BLUE).apply();
        Thread.sleep(1000);

        ledRiver.setColor(Color.GREEN).apply();
        Thread.sleep(1000);

        ledRiver.setColor(Color.YELLOW).apply();
        Thread.sleep(1000);

        ledRiver.setMode(LEDRiver.Mode.PATTERN).setColor(Color.BLUE);
        ledRiver.setPattern(LEDRiver.Pattern.STROBE.builder().setSpeed(1000));
        ledRiver.apply();
        Thread.sleep(5000);

        ledRiver.setPattern(LEDRiver.Pattern.HEARTBEAT.builder()).apply();
        Thread.sleep(5000);

        ledRiver.setPattern(LEDRiver.Pattern.BREATHING.builder()).apply();
        Thread.sleep(5000);

        ledRiver.setPattern(LEDRiver.Pattern.RUNNING.builder()).apply();
        Thread.sleep(5000);

        ledRiver.setPattern(LEDRiver.Pattern.BOUNCING.builder().setSpeed(700)).apply();
        Thread.sleep(5000);

        ledRiver.setPattern(LEDRiver.Pattern.THEATRE_RUNNING.builder()).apply();
        Thread.sleep(5000);

        ledRiver.setPattern(LEDRiver.Pattern.THEATRE_BOUNCING.builder()).apply();
        Thread.sleep(5000);

        ledRiver.setPattern(LEDRiver.Pattern.COLOR_WHEEL.builder()).apply();
        Thread.sleep(5000);


    }
}
