package org.firstinspires.ftc.teamcode.PSRobotLibs.lib.auto;

import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.PSResources;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.drive.PSDrive;

/**
 * Created by young on 8/6/2017.
 */

public class PSAutoDrive {

    private PSResources resources;

    private PSDrive drive;

    /**
     * Constructor of the WorldAuto drive class
     *
     * @param res   Passes the resources for use in methods
     * @param drive gives direct access to the drive methods for ease of use
     */
    public PSAutoDrive(PSResources res, PSDrive drive) {
        resources = res;
        this.drive = drive;
    }

}