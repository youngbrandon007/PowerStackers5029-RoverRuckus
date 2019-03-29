package org.firstinspires.ftc.teamcode.roverRuckus.Robot_r5;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Cluster")
public class ClusterDetection extends LinearOpMode{

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AS+d9VT/////AAABmctzt2QKCkCHsWckqSrT1KQGWoYW3xOlwF6eKDby0X3s3w/cOZDacTUq9nSp07w99XYiDRBwazpKsyrGpb91tabfndtOpx9EmRnJXteDwU8VXOnWysQIG8PBqHCXmOTYwXCCcJtp6KQIDjw9PrJDlUAit/rWlA5pLD8UdFyFkJiTNaInnUxsRqQQSeUMxKGuf4QcHDh6g72cZ9GjrCH+hdgIFJ7mta711jglu1a8kMKHSD9XbBOhD6DK9tATWVSsVEO4DCuKz61TRpCuqfSc62yXHkWDHanBSG9ca99r+TVrwKE7QCzMDkLSrKW/KHSr+MNPSTuCGnWBGNL4QSmsJ6ec6IqINx7IYWFeUMyFGM4w";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        ArrayList<Mineral> minerals = new ArrayList<>();

                        for(Recognition rec:updatedRecognitions){
                            double mx = 0;
                            double my = 0;
                            double mz = (rec.getLabel().equals(LABEL_GOLD_MINERAL) ? 1 : 1.375);
                            double cx = 0;
                            double cy = 0;
                            double cz = 5.375;
                            double crx = 135;
                            double cry = 0; // 0 is looking at wall
                            double cpw = rec.getImageWidth();
                            double cph = rec.getImageHeight();
                            double cfovax = 70.42;
                            double cfovay = 43.3;
                            double rx = 0;
                            double ry = 0;

                            double mpx = ((rec.getRight() + rec.getLeft()) / 2) - (cpw / 2);
                            double mpy = ((rec.getTop() + rec.getBottom()) / 2) - (cpw / 2);

                            telemetry.addData("x", mpx);
                            telemetry.addData("y", mpy);

                            double ax = (cfovax* mpx/cpw);
                            double ay = -((-cfovay* mpy/cph)-20);

                            telemetry.addData("ax", ax);
                            telemetry.addData("ay", ay);

                            double relxdis = Math.tan(Math.toRadians((90 - cry) - ay)) * (cz - mz); // relative x distance
                            double relydis = Math.tan(Math.toRadians(ax)) * (relxdis);
                            telemetry.addData("relx", relxdis);
                            telemetry.addData("rely", relydis);

                            mx = cx - (Math.sin(Math.toRadians(crx) * relxdis));
                            mx = mx + (Math.cos(Math.toRadians(crx - 90) * relydis));

                            my = cy - (Math.cos(Math.toRadians(crx) * relxdis));
                            my = my - (Math.sin(Math.toRadians(crx - 90) * relydis));

                            telemetry.addData("COORD x", mx);
                            telemetry.addData("COORD y", my);

                            minerals.add(new Mineral(rec, mx , my,  mz));
                        }

                        telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}

