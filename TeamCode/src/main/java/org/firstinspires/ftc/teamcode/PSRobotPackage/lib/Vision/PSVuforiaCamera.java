package org.firstinspires.ftc.teamcode.PSRobotPackage.lib.Vision;

import com.vuforia.CameraCalibration;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.PSRobotPackage.lib.PSEnum;
import org.firstinspires.ftc.teamcode.PSRobotPackage.lib.PSRobotConstants;

public class PSVuforiaCamera extends PSCamera {
    public VuforiaLocalizer vuforia;


    public PSVuforiaCamera(PSEnum.CameraDirection cameraDirection) {
        super(cameraDirection);

    }

    @Override
    protected void doInitialize() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = PSRobotConstants.VUFORIA_KEY;
        parameters.cameraDirection = (cameraDirection== PSEnum.CameraDirection.BACK)?VuforiaLocalizer.CameraDirection.BACK: VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.createVuforiaLocalizer(parameters);

    }

    @Override
    public void close() {

    }

    @Override
    public Properties getProperties() {
        return new VuforiaProperties();
    }
    private class VuforiaProperties implements Properties {
        @Override
        public double getHorizontalFocalLengthPx(double imageWidth) {
            CameraCalibration cameraCalibration = CameraDevice.getInstance().getCameraCalibration();
            double fov = cameraCalibration.getFieldOfViewRads().getData()[0];
            return (imageWidth * 0.5) / Math.tan(0.5 * fov);
        }
    }

}
