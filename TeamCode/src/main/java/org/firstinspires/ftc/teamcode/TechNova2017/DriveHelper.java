package org.firstinspires.ftc.teamcode.TechNova2017;

import android.util.Log;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.DPAD_SCALE_TO_DRIVE;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.LOW_MODE_SCALE_TO_DRIVE;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.VERY_LOW_MODE_SCALE_TO_DRIVE;

/**
 * Gamepad mapping and mecanum drive input control algorithms
 *
 * There are 4 different drive mode
 *
 *    1. Standard drive
 *    2. DPAD drive (4 directions at 30% max speed)
 *    3. Slow mode drive (at 65% max speed)
 *    4. Very slow mode drive (at 45% max speed)
 *
 *  The scaleToDrive parameter can be changed at {@link RobotInfo}
 */
public class DriveHelper {
    static boolean slowDrive = false;
    static boolean verySlowDrive = false;
    static boolean inverted = false;
    static double scaleToDrive = 1.0;

    // assumes that the controller is updated
    static void drive(Controller g, MecanumRobot robot, Telemetry telemetry) {
        double theta = 0.0, v_theta = 0.0, v_rotation = 0.0;

        final double dpad_speed = DPAD_SCALE_TO_DRIVE;

        double lx = 0.0;
        double ly = 0.0;

        if(g.YOnce()) {
            //arcadeDrive = !arcadeDrive;
            //inverted = !inverted;
        }

        if(g.leftBumper() && g.XOnce()) {
            verySlowDrive = !verySlowDrive;
            if(verySlowDrive) {
                scaleToDrive = VERY_LOW_MODE_SCALE_TO_DRIVE;
            } else {
                scaleToDrive = 1.0;
            }
        }
        else if(g.rightBumperOnce() || (g.leftBumperOnce() && g.rightBumperOnce())) {
            slowDrive = !slowDrive;
            if(slowDrive) {
                scaleToDrive = LOW_MODE_SCALE_TO_DRIVE;
            } else {
                scaleToDrive = 1.0;
            }
        }

        if (g.dpadUp()) {
            theta = 0.0;
            v_theta = dpad_speed;
        }
        else if (g.dpadDown()) {
            theta = Math.PI;
            v_theta = dpad_speed;
        }
        else if (g.dpadLeft()) {
            theta = 3.0 * Math.PI / 2.0;
            v_theta = dpad_speed;
        }
        else if (g.dpadRight()) {
            theta = Math.PI / 2.0;
            v_theta = dpad_speed;
        }
        else {
            lx = Math.pow(g.left_stick_x, 3.0)*scaleToDrive;
            ly = -Math.pow(g.left_stick_y, 3.0)*scaleToDrive;

            theta = Math.atan2(lx, ly);
            v_theta = Math.hypot(lx,ly);
            v_rotation = Math.pow(g.right_stick_x, 3.0)*scaleToDrive;

            v_rotation = Range.clip(v_rotation,-1.0,1.0);
        }

        telemetry.addData("Joystick: ", String.format("%.2f %.2f %.2f", g.left_stick_x, g.left_stick_y, g.right_stick_x));
        telemetry.addData("Slow Mode: ", slowDrive + " | " + String.format("%.2f",scaleToDrive));
        telemetry.addData("Very slow Mode: ", verySlowDrive + " | " + String.format("%.2f",scaleToDrive));

        telemetry.addData("Glyph Lift Position: ", robot.getGlyphLiftPosition());

        robot.logMotorEncoders(telemetry, false);

        // If A or B are pressed, rotate drive motion by 90 degrees for more effective
        // teleop button pushing.
//        if (g.leftBumper() || g.rightBumper()) {
//            theta += Math.PI / 2.0;
//        }

        if(inverted) {
            theta += Math.PI;
            theta %= 2*Math.PI;
        }
        robot.drive(theta, v_theta, v_rotation);

        if(slowDrive) {
            robot.turnOnGreenLed();
        } else {
            robot.turnOffGreenLed();
        }
    }

    public static void logInfo(Telemetry telemetry, String tag, String message) {
        Log.i("DriverHelper", tag + " | " + message);

        if(telemetry != null) {
            telemetry.addData(tag, message);
        }
    }
}
