package org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.Vuforia;

import com.vuforia.HINT;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.PineappleRobotPackage.lib.PineappleResources;
import org.firstinspires.ftc.teamcode.R;

/**
 * Created by young on 9/12/2017.
 */

public class PineappleVuforia {

    PineappleResources resources;

//    PineappleVuforiaLocalizerImplSubclass vuforia;
    VuforiaLocalizer vuforia;
    VuforiaTrackables targets;

    public void addResources(PineappleResources r){
        resources = r;
    }

    public PineappleVuforia(int maxItemCount, VuforiaLocalizer.CameraDirection direction, VuforiaLocalizer.Parameters.CameraMonitorFeedback feedback, String vuforiaLicenseKey){

        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = direction;
        params.vuforiaLicenseKey = vuforiaLicenseKey;
        params.cameraMonitorFeedback =  feedback;

//        vuforia = new PineappleVuforiaLocalizerImplSubclass(params);
        vuforia = ClassFactory.createVuforiaLocalizer(params);;
        vuforia.setFrameQueueCapacity(1);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS,maxItemCount);
    }

    public void addTrackables(String assetName, String names[]){
        targets = vuforia.loadTrackablesFromAsset(assetName);
        for(int i=0; i < names.length; i++){
            targets.get(i).setName(names[i]);
        }
    }

    public void startTracking(){
        targets.activate();
    }

    public void showTrackData(){

        for(VuforiaTrackable target : targets) {
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) target.getListener()).getPose();
            if (pose != null) {
                resources.feedBack.sayFeedBackWithOutUpdate("Pose", format(pose));
            }
        }
        resources.feedBack.update();
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
