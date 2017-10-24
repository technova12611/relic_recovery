
package org.firstinspires.ftc.teamcode.TechNova2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.concurrent.TimeUnit;

@Autonomous(name="VuMark Id", group ="Test")
@Disabled
public class VuMarkTest extends LinearOpMode {

    public static final String TAG = "Vuforia VuMark Test";

    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    @Override public void runOpMode() {

        VuMarkVision vuMarkVision = new VuMarkVision(hardwareMap, telemetry);
        vuMarkVision.activate();

        boolean flashOn = true;
        vuMarkVision.setFlashTorchMode(flashOn);

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive()) {
            vuMarkVision.detect(telemetry);

            if(timer.time(TimeUnit.SECONDS) % 5 == 0) {
                vuMarkVision.setFlashTorchMode(!flashOn);
            }
        }
    }

}
