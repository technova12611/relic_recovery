package org.firstinspires.ftc.teamcode.TechNova2017;

import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by terryw on 10/13/2017.
 */

public class PositionDetector {
    ModernRoboticsI2cRangeSensor xDistanceSensor;
    ModernRoboticsI2cRangeSensor yDistanceSensor;

    double previousXDistance = 0.0;
    double previousYDistance = 0.0;

    public PositionDetector(HardwareMap hwMap, Telemetry telemetry) {
        try {
            this.xDistanceSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "xRange");
            this.xDistanceSensor.getDistance(DistanceUnit.CM);

            telemetry.addData ("06", "X Range sensor initialized ****");
            Log.i(this.getClass().getSimpleName(), "xRange sensor initialized");
        }
        catch (Exception e)
        {
            telemetry.addData("06.", "X Range sensor failed");
            Log.e(this.getClass().getSimpleName(), "Exception: " + e.getMessage());
            this.xDistanceSensor = null;
        }

        try {
            this.yDistanceSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "yRange");
            this.yDistanceSensor.getDistance(DistanceUnit.CM);

            telemetry.addData ("06", "Y Range sensor initialized ****");
            Log.i(this.getClass().getSimpleName(), "yRange sensor initialized");
        }
        catch (Exception e)
        {
            telemetry.addData("06.", "Y Range sensor failed");
            Log.e(this.getClass().getSimpleName(), "Exception: " + e.getMessage());
            this.yDistanceSensor = null;
        }
    }

    public double getXDistance() {

        if(xDistanceSensor != null) {
            double distance = xDistanceSensor.getDistance(DistanceUnit.CM);
            if(distance < 200.0) {
                previousXDistance = distance;
            }
            return previousXDistance;
        }
        return 0.0;
    }

    public double getYDistance() {

        if(yDistanceSensor != null) {
            double distance = yDistanceSensor.getDistance(DistanceUnit.CM);
            if(distance < 200.0) {
                previousYDistance = distance;
            }
            return previousYDistance;
        }
        return 0.0;
    }
}
