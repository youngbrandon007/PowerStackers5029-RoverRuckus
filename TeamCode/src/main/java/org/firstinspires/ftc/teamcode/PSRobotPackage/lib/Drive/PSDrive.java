package org.firstinspires.ftc.teamcode.PSRobotPackage.lib.Drive;

import org.firstinspires.ftc.teamcode.PSRobotPackage.lib.PSResources;

/**
 * Created by ftcpi on 6/29/2017.
 */

public class PSDrive extends PSDriveAbstract {

    public PSTankDrive tank;
    public PSMecanumDrive mecanum;

    public PSDrive(PSResources res) {
        super(res);
        resources = res;
        tank = new PSTankDrive(resources);
        mecanum = new PSMecanumDrive(resources);
    }

}