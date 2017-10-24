package org.firstinspires.ftc.teamcode.ETC2016OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by terryw on 7/19/2016.
 */
public abstract class ETCBaseOpMode extends OpMode {

    /**
     * Scale the joystick input using READ_MODE nonlinear algorithm.
     */
    public float scaleMotorPower (float p_power)
    {
        //
        // Assume no scaling.
        //
        float scale = 0.0f;
        //
        // Ensure the values are legal.
        //
        float power = Range.clip (p_power, -1, 1);
        float[] array =
                { 0.00f, 0.05f, 0.07f, 0.09f, 0.12f
                        , 0.15f, 0.20f, 0.24f, 0.30f, 0.36f
                        , 0.40f, 0.45f, 0.50f, 0.60f, 0.75f
                        , 0.85f, 1.00f
                };
        //
        // Get the corresponding index for the specified argument/parameter.
        //
        int index = (int)(power * 16.0);
        if (index < 0)
        {
            index = -index;
        }
        else if (index > 16)
        {
            index = 16;
        }

        if (power < 0)
        {
            scale = -array[index];
        }
        else
        {
            scale = array[index];
        }

        return scale;
    }

    public float scaleMotorPowerSlowMode (float p_power)
    {
        //
        // Assume no scaling.
        //
        float scale = 0.0f;
        //
        // Ensure the values are legal.
        //
        float power = Range.clip (p_power, -1, 1);
        float[] array =
                { 0.00f, 0.05f, 0.07f, 0.09f, 0.10f
                        , 0.10f, 0.12f, 0.15f, 0.18f, 0.20f
                        , 0.22f, 0.25f, 0.28f, 0.30f, 0.32f
                        , 0.35f, 0.40f
                };
        //
        // Get the corresponding index for the specified argument/parameter.
        //
        int index = (int)(power * 16.0);
        if (index < 0)
        {
            index = -index;
        }
        else if (index > 16)
        {
            index = 16;
        }

        if (power < 0)
        {
            scale = -array[index];
        }
        else
        {
            scale = array[index];
        }

        return scale * (float)(Math.abs(p_power) > 0.6?1.25:1.0);
    }

    /**
     * Update the telemetry with current gamepad readings.
     */
    public void updateGamepadTelemetry ()
    {
        //
        // Send telemetry data concerning gamepads to the driver station.
        //
        telemetry.addData ("01", "GP1 Left: " + gamepad1.left_stick_y);
        telemetry.addData ("02", "GP1 Right: " + gamepad1.right_stick_y);
        telemetry.addData ("03", "GP2 Left: " + gamepad2.left_stick_y);
        telemetry.addData ("04", "GP2 Right: " + gamepad2.right_stick_y);
        telemetry.addData ("05", "GP2 Left Trig: " + gamepad2.left_trigger);
        telemetry.addData ("06", "GP2 Right Trig: " + gamepad2.right_trigger);
        telemetry.addData ("07", "GP1 A: " + gamepad1.a);
        telemetry.addData ("08", "GP1 B: " + gamepad1.b);
    }

    protected void delay(long milliSecs){
        try {
            Thread.sleep(milliSecs);
        } catch (InterruptedException e){}
    }

    public enum AllianceColor {
        RED,
        BLUE
    }

    public abstract AllianceColor getAllianceColor();
}
