package org.firstinspires.ftc.teamcode.TechNova2017;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Follow {@link ConceptVuMarkIdentification}, restructure to fit in our code
 */
public class VuMarkVision {

    VuforiaLocalizer vuforia;

    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;

    boolean useFrontCamera = true;
    boolean turnOnFlash = false;

    public VuMarkVision(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, true);
    }

    public VuMarkVision(HardwareMap hardwareMap, Telemetry telemetry, boolean frontCamera) {

        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

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
         * and paste it in to your code onthe next line, between the double quotes.
         */
        parameters.vuforiaLicenseKey = "Af9uAMv/////AAAAGTbSRmmcV0REnxcQRuO3G0VvPwOcogiLI5jL0wWQmYFswf7LAr064ZZXPKp3BW//mwOo2qguEedg3DCLf6U1zPwmxINewBq48HY6NaFI9qTR0rMPEj7erdH5FrdLLrKb5UJfrY24gQ1oDiVWOM+bfdi2mWz8MeiMIyJmAuiLR5xEulOThAP/gp4X6kUBKH6XDxlPXmRR/NXjcPiv0xKrT+9bTnTP1c7F2kWYl9RVILTJmclz0p71ndspBtAzuFCvpNeB2JCxbaayCYZtdfGt6Be2BbEFOVY/4kNps4sy75Em/VSgfkOjHofEFeXx7ECT9gq2RkUhs/ki9X2sCrQ2cTUNxxQyNNRlAH5BY3dPuU5J";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        if(frontCamera) {
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        } else {
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
            setFlashTorchMode(true);
        }
        this.useFrontCamera = frontCamera;

        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        this.relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        this.relicTemplate = relicTrackables.get(0);
        this.relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData("VuMark", "VuMark is Ready");
        telemetry.update();
    }

    public void activate() {
        relicTrackables.activate();
    }

    public RelicRecoveryVuMark detect (Telemetry telemetry) {
        return detect(telemetry, true);
    }

    public void setFlashTorchMode(boolean flashOn) {
        turnOnFlash = flashOn;
    }

    public RelicRecoveryVuMark detect (Telemetry telemetry, boolean doLog) {
        /**
         * See if any of the instances of {@link relicTemplate} are currently visible.
         * {@link RelicRecoveryVuMark} is an enum which can have the following values:
         * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
         * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
         */

        if(turnOnFlash) {
            CameraDevice.getInstance().setFlashTorchMode(true);
        }

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        String message = String.format("%s visible", vuMark);

        if(telemetry != null) {
            telemetry.addData("VuMark", message);
            telemetry.update();
        }

        if(doLog) logInfo("VuMark", message);

        if(vuMark != RelicRecoveryVuMark.UNKNOWN) {
            setFlashTorchMode(false);
        }
        return vuMark;
    }

    public void logInfo(String tag, String message) {
        Log.i(this.getClass().getSimpleName(), tag + " | " + message);
    }
}
