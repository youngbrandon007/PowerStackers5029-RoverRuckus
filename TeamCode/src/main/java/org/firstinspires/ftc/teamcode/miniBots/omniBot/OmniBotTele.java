package org.firstinspires.ftc.teamcode.miniBots.omniBot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "OmniTele", group = "omnibot")
@Disabled
public class OmniBotTele extends OmniBotConfig {
    boolean thing = false;

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
    }

    @Override
    public void loop() {
        telemetry.addData("Gyro", gyroSensor.getHeading());
        if (thing) {
            robot.drive.mecanum.updateMecanumThirdPerson(gamepad1, 1, Math.toRadians(gyroSensor.getHeading()));
        } else {
            robot.drive.mecanum.updateMecanum(gamepad1, 1);
        }
        if (gamepad1.left_bumper) {
            thing = true;
        } else if (gamepad1.right_bumper) {
            thing = false;
        }
        telemetry.update();
    }
}
