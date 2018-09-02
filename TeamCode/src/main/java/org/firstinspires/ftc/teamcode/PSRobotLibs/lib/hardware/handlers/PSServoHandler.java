package org.firstinspires.ftc.teamcode.PSRobotLibs.lib.hardware.handlers;

import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSEnum;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.hardware.PSServo;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSResources;

/**
 * Created by Brandon on 9/13/2017.
 */

public class PSServoHandler {

    private PSResources resources;

    public PSServoHandler(PSResources r) {
        resources = r;
    }

    public PSServo newServo(String name, double totalRotation, double init, boolean goToInit) {
        return new PSServo(resources,name,totalRotation,init,goToInit);
    }

    public PSServo newServo(String name, PSEnum.ServoTotalRotation servoTotalRotation, double init, String[] names, double[] poses, boolean goToInit) {
        return new PSServo(resources,name,servoTotalRotation,init,names, poses,goToInit);
    }

}
