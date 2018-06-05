package org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.Drive;

import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleResources;

/**
 * Created by ftcpi on 6/29/2017.
 */

public class PineappleDrive extends PineappleDriveAbstract {

    public PineappleTankDrive tank;
    public PineappleMecanumDrive mecanum;

    public PineappleDrive(PineappleResources res) {
        super(res);
        resources = res;
        tank = new PineappleTankDrive(resources);
        mecanum = new PineappleMecanumDrive(resources);
    }

}