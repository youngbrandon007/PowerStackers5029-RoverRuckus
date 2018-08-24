package org.firstinspires.ftc.teamcode.PSRobotPackage.lib.Auto;

import android.os.Environment;
import android.util.Log;

import org.firstinspires.ftc.teamcode.PSRobotPackage.lib.Drive.PSDrive;
import org.firstinspires.ftc.teamcode.PSRobotPackage.lib.PSResources;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;

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