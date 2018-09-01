package org.firstinspires.ftc.teamcode.PSRobotLibs.lib.hardware.handlers;

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

    public PSServo newLimitServo(String name, double pos, double init) {
//        return new PSServo(resources, name, , pos, init, true);
        return new PSServo(resources,name,100,init,false);
    }

    public PSServo newContinuousServo(String name, double init){
//        return new PSServo(resources, name, PSEnum.ServoType.CONTINUOUS, 1, init, true);
        return new PSServo(resources,name,100,init,false);
    }
}
