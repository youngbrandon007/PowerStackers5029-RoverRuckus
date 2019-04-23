package org.firstinspires.ftc.teamcode.roverRuckus.Robot_r5;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.hardware.LEDRiver;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.vision.UVC.UVCCamera;
import org.firstinspires.ftc.teamcode.Paths.PathGenerator;
import org.firstinspires.ftc.teamcode.Paths.PathPoints;
import org.firstinspires.ftc.teamcode.roverRuckus.Robot_r5.Transition.AutoTransitioner;
import org.opencv.android.OpenCVLoader;

@TeleOp(name = "r5.PreAuto", group = "r5")
public class PreAuto extends Config_r5{

    @Override
    public void init() {
        //load data from auto app

        //init robot
        config(this);

    }

    @Override
    public void start(){

        AutoTransitioner.transitionOnStop(this, "r5.Auto");

//        lights.load();
//
//        lights.solid(new LEDRiver.Color(0, 255, 0, 0));
//        lights.setBrightness(7);
        drive.unreleaseMarker();
    }

    @Override
    public void loop() {
        robot.drive.mecanum.updateMecanum(gamepad1, 1.0);

        if(gamepad1.right_trigger > .5){
            drive.releaseMarker();
        }else{
            drive.unreleaseMarker();
        }

        if(gamepad2.right_stick_x > .8){
            lift.bridge.openBridge();
            telemetry.addData("S","Open");
        }else if(gamepad2.right_stick_x < -.8){
            lift.bridge.closeBridge();
            telemetry.addData("S","Close");
        }else{
            lift.bridge.stopBridge();
            telemetry.addData("S","Stop");
        }

        if(gamepad2.left_stick_x > .8){
            lift.bridge.canopy.setPosition(0);
        }else if(gamepad2.left_stick_x < -.8){
            lift.bridge.canopy.setPosition(1.0);
        }

        if(gamepad1.left_bumper || gamepad2.left_bumper){
            lift.bridge.doorServo.setPosition(1);
        }else{
            lift.bridge.doorServo.setPosition(.67);
        }

        if(gamepad1.x){
            setColor("blue");
            //lights.loadTeamColor();
            telemetry.addData("Change", "blue");
        }
        if(gamepad1.b){
            setColor("red");
            //lights.loadTeamColor();
            telemetry.addData("Change", "red");
        }

        //telemetry.addData("Team Color",  "-" + lights.allianceColor + "-");

        //lights.setTeamColor();

        lift.extension.setPower((gamepad1.dpad_up || gamepad2.dpad_up) ? -1.0 : (gamepad1.dpad_down || gamepad2.dpad_down) ? 1.0 : 0.0);
        collector.extension.setPower(((gamepad2.right_bumper) ? -1 : ((gamepad2.left_bumper) ? 1 : 0))+gamepad1.right_stick_y);

    }

}