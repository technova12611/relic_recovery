package org.firstinspires.ftc.teamcode.ETC2016OpModes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Color Sensor Test", group = "sensor")
@Disabled
public class ETCColorSensorTest extends OpMode
{
    protected long elapsedTime = 0;

    private ColorSensor colorSensor;

    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    static final int    DIM_BLUE_LED    = 0;     // Blue LED channel on DIM
    static final int    DIM_RED_LED     = 1;     // Red LED Channel on DIM

    DeviceInterfaceModule dim;
    public ETCColorSensorTest()
    {
    }

    @Override
    public void init() {

        initColorSensor();

        telemetry.update();

        dim = hardwareMap.deviceInterfaceModule.get("dim");
    }

    @Override
    public void init_loop() {
        if(dim != null) {
            boolean even = (((int)(runtime.time()) & 0x01) == 0);
            dim.setLED(DIM_RED_LED,   even); // Red for even
            dim.setLED(DIM_BLUE_LED, !even); // Blue for odd
        }
    }
    /**
     * Perform any actions that are necessary when the OpMode is enabled.
     *
     * The system calls this member once when the OpMode is enabled.
     */
    @Override public void start ()
    {
        runtime.reset();
    } // start


    @Override public void loop ()
    {
        telemetry.addData ("00", runtime.seconds());
        runtime.log("Test Sensor :");
        try {
            if(colorSensor != null) {
                telemetry.addData("40", "Color    Red: " + colorSensor.red() + " Blue: " + colorSensor.blue());
            }
            else {
                telemetry.addData("50", "Bad Sensor ...");
            }
        }catch(Exception e) {
            // ignore
        }
        telemetry.update();
    } // loop

    protected void initColorSensor() {
        try
        {
            colorSensor = hardwareMap.colorSensor.get("colorSensorBeacon");
            colorSensor.enableLed(true);

            if(colorSensor.red() == 255) {
                colorSensor = null;
                String error = " ** Color Sensor Failed. " + " Stop and Restart !!!";
                telemetry.addData("04. Color: ", error);

                Log.e(this.getClass().getName(), error);
            }

            telemetry.addData ("02.", "Color Initialized");
            logInfo("color sensor initialized");
        }
        catch (Exception e)
        {
            telemetry.addData("03.", "color sensor  failed");
            Log.e(this.getClass().getSimpleName(), "Exception: " + e.getMessage());
            colorSensor = null;
        }
    }

    protected void delay(long milliSecs){
        try {
            Thread.sleep(milliSecs);
        } catch (InterruptedException e){}
    }

    protected void logInfo(String message) {
        Log.i(this.getClass().getSimpleName(), message);
    }
}

