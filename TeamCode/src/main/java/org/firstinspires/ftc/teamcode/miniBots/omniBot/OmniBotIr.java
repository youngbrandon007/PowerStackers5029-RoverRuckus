package org.firstinspires.ftc.teamcode.miniBots.omniBot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
@TeleOp(name="OmniIr", group="omnibot")
@Disabled
public class OmniBotIr extends OmniBotConfig {
    IrSeekerSensor ir;
    double irOn = 0;
    @Override
    public void init() {
        config(this);
        gyroSensor.calibrate();
        while (gyroSensor.isCalibrating()) {
            telemetry.addData("Gyro", "Calib");
            telemetry.update();
        }
        telemetry.addData("Gyro", "Done");
        telemetry.update();
        ir = hardwareMap.irSeekerSensor.get("IR");
    }

    @Override
    public void loop() {
        telemetry.addData("IR", ir.signalDetected());
        telemetry.addData("IR Angle", ir.getAngle());
        telemetry.addData("IR Power", ir.getStrength());
        telemetry.addData("IR Mode", irOn);
        telemetry.addData("Gyro", gyroSensor.getHeading());
        if (irOn == 1) {
            robot.drive.mecanum.setMecanumThridPerson(Math.toRadians(ir.getAngle()+90), 0.03*(255-ir.getStrength()),0.05*(gyroSensor.getHeading()-ir.getAngle()),1,Math.toRadians(gyroSensor.getHeading()));
        } else if (irOn == 2){
            if (ir.getAngle()>-20&&ir.getAngle()<20){
                if (ir.getAngle()>0){
                    robot.drive.tank.setPower(0.05,0.05);
                }else{
                    robot.drive.tank.setPower(-0.05,-0.05);
                }
            }
        else if(ir.getStrength()<240){
            robot.drive.tank.setPower(-0.2,0.2);

        }
            else{
                robot.drive.tank.setPower(0,0);
            }

        }
        if (gamepad1.a) {
            irOn = 0;
        } else if (gamepad1.b) {
            irOn = 1;
        } else if (gamepad1.x){
            irOn = 2;
        }
        telemetry.update();
    }
}
