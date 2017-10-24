package org.firstinspires.ftc.teamcode.ETC2016OpModes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceReader;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.ETCHardware.ETCAdafruitIMU;

@TeleOp(name = "Sensor Test", group = "Test")
@Disabled
public class ETCSensorTest extends OpMode
{
    protected long elapsedTime = 0;

    protected I2cDeviceReader rangeReader;
    protected I2cDeviceSynchImpl rangeReaderSync;

    private ETCAdafruitIMU gyroSensor;
    private ColorSensor colorSensorBottom;

    private ElapsedTime runtime = new ElapsedTime();

    static final int    DIM_BLUE_LED    = 0;     // Blue LED channel on DIM
    static final int    DIM_RED_LED     = 1;     // Red LED Channel on DIM

    DeviceInterfaceModule dim;
    public ETCSensorTest()
    {
    }

    @Override
    public void init() {
        initGyroSensor();

        initColorSensor();

        initRangeSensor();

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
        telemetry.addData ("10", "US  Distance: "  +  getRangeSensorDistance()[0]);
        telemetry.addData ("20", "ODS Distance: "  +  getRangeSensorDistance()[1]);

        try {
            telemetry.addData("30", "Gyro Heading: " + gyroSensor != null ? gyroSensor.getHeadingAngle() : "0");
            telemetry.addData("40", "Color    Red: " + colorSensorBottom != null ? colorSensorBottom.red() : "0");
        }catch(Exception e) {
            // ignore
        }
        telemetry.update();
    } // loop

    public int[] getRangeSensorDistance() {
        int distance[] = {0, 0};
        if(rangeReaderSync != null) {
            //byte rangeReadings[] = rangeReader.getReadBuffer();
            byte rangeReadings[] = rangeReaderSync.read(0x04, 2);
            distance[0] = (rangeReadings[0] & 0xFF);
            distance[1] = (rangeReadings[1] & 0xFF);
        }
        return distance;
    }

    /**
     * handle AdaFruit BNO055 IMU sensor initialization
     */
    protected long initGyroSensor() {
        long systemTime = System.currentTimeMillis();
        double headingAngle = 0.0;

        try {
            gyroSensor = new ETCAdafruitIMU(hardwareMap, "gyroSensor"
                    , (byte)(ETCAdafruitIMU.BNO055_ADDRESS_A * 2)
                    , (byte)ETCAdafruitIMU.OPERATION_MODE_IMU);

            delay(50);

            gyroSensor.startIMU();
            headingAngle = gyroSensor.getHeadingAngle();
        }
        catch (Exception e){
            gyroSensor = null;
            Log.e(this.getClass().getName(), "Exception: " + e.getMessage());
        }

        long elapsedTime = System.currentTimeMillis() - systemTime;
        String initStatus = (gyroSensor == null?" | Failed, Stop and Restart": " | Success");
        logInfo("Gyro sensor initialized in: " +
                elapsedTime + " ms." + " | Heading: " + headingAngle + initStatus);

        telemetry.addData ("01. IMU init: ",  elapsedTime
                + "  | Heading: " + headingAngle + initStatus);

        return elapsedTime;
    }

    protected void initColorSensor() {
        try
        {
            colorSensorBottom = hardwareMap.colorSensor.get("colorSensorBottomFront");
            colorSensorBottom.enableLed(true);

            if(colorSensorBottom.red() == 255) {
                colorSensorBottom = null;
                String error = " ** Bottom Color Sensor Failed. " + " Stop and Restart !!!";
                telemetry.addData("04. Color: ", error);

                Log.e(this.getClass().getName(), error);
            }

            telemetry.addData ("02.", "Color Bottom Initialized");
            logInfo("color sensor bottom initialized");
        }
        catch (Exception e)
        {
            telemetry.addData("03.", "color sensor bottom failed");
            Log.e(this.getClass().getSimpleName(), "Exception: " + e.getMessage());
            colorSensorBottom = null;
        }

        delay(50);
    }

    protected void initRangeSensor() {
        try
        {
            rangeReaderSync = new I2cDeviceSynchImpl(hardwareMap.i2cDevice.get("rangeSensor"),
                    I2cAddr.create8bit(0x28), false);

            rangeReaderSync.setReadWindow(new I2cDeviceSynch.ReadWindow(0x04,2, I2cDeviceSynch.ReadMode.REPEAT));
            rangeReaderSync.engage();

            Log.i("ETCRangeSensorTest", "range sensor initialized ****");
            telemetry.addData ("00", "range sensor initialized ****");
        }
        catch (Exception ex)
        {
            RobotLog.e (ex.getLocalizedMessage ());
            rangeReaderSync = null;
            telemetry.addData ("00", "Range Sensor Init failed.");
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

