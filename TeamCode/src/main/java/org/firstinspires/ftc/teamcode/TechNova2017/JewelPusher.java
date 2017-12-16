package org.firstinspires.ftc.teamcode.TechNova2017;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.TechNova2017.JewelPusher.Color.BLUE;
import static org.firstinspires.ftc.teamcode.TechNova2017.JewelPusher.Color.RED;
import static org.firstinspires.ftc.teamcode.TechNova2017.JewelPusher.Color.UNKNOWN;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.JEWEL_PUSHER_LONG_ARM_HALF_TARGET_POSITION;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.JEWEL_PUSHER_LONG_ARM_INITIAL_POSITION;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.JEWEL_PUSHER_LONG_ARM_TARGET_POSITION;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.JEWEL_PUSHER_LONG_ARM_TELEOPS_POSITION;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.JEWEL_PUSHER_SHORT_ARM_INITIAL_POSITION;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.JEWEL_PUSHER_SHORT_ARM_LEFT_PUSH_POSITION;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.JEWEL_PUSHER_SHORT_ARM_RIGHT_PUSH_POSITION;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.JEWEL_PUSHER_SHORT_ARM_STRAIGHT_POSITION;

/** need to define two servos and one color sensor
 * should have methods to
 *    1. move the arms in right position at start
 *        arm servo 1 should lower the long arm down
 *       arm servo 2 should rotate the short arm to straight position
 *  2. once detect the color, move the second
 *  3. move arms back up
 *        arm servo 1 should raise up the long arm
 *        arm servo 2 should rotate the short arm to original position
*/
public class JewelPusher {

    Servo longArm;
    Servo shortArm;
    AllianceColor alliance = AllianceColor.RED;

    ColorSensor jewelColor;

    Telemetry _telemetry = null;

    LinearOpMode _autoOpMode;

    /**
     * initialize servos and color sensor
     */
    public JewelPusher(LinearOpMode autoOp, HardwareMap hardwareMap, Telemetry telemetry, AllianceColor alliance, boolean isAuto)
            throws InterruptedException {

        _telemetry = telemetry;
        _autoOpMode = autoOp;

        longArm = hardwareMap.servo.get("longArm");

        // make sure long arm is in the up right position
        longArm.setPosition(JEWEL_PUSHER_LONG_ARM_INITIAL_POSITION);

        shortArm = hardwareMap.servo.get("shortArm");

        // make sure short arm is in folding position
        shortArm.setPosition(JEWEL_PUSHER_SHORT_ARM_INITIAL_POSITION);

        if(isAuto) {
            this.alliance = alliance;

            jewelColor = hardwareMap.colorSensor.get("jewelColor");
            //jewelColor.enableLed(true);

            waitForMS(200);

            telemetry.addData("Color: ", jewelColor.argb());
            Log.i(this.getClass().getSimpleName(), "Color in Init (R,G, B):"
                    + jewelColor.red() + ", " + jewelColor.green() + ", " + jewelColor.blue());
        }
    }

    /**
     * low the long arm and rotate the short arm
     *   servo.setPosition()
     */
    public void extend() throws InterruptedException {

        Log.i(this.getClass().getSimpleName(), "Deploying jewel pusher arms ...");

        // long arm down
        //--------------------------------------
        longArm.setPosition(JEWEL_PUSHER_LONG_ARM_HALF_TARGET_POSITION);

        waitForMS(750);

        // set to the correct position (middle position)
        // then wait a bit
        shortArm.setPosition(JEWEL_PUSHER_SHORT_ARM_STRAIGHT_POSITION);

        // wait for ~ 1.0 seconds
        waitForMS(750);

        // long arm down
        //--------------------------------------
        longArm.setPosition(JEWEL_PUSHER_LONG_ARM_TARGET_POSITION);

        // wait for 2 seconds
        waitForMS(1000);
    }

    /**
     * Move the long arm back to upright position
     * short arm should be still in folding position after pushing the jewel
     */
    public void retract() throws InterruptedException {

        Log.i(this.getClass().getSimpleName(), "Retracting jewel pusher arms ...");

        longArm.setPosition(JEWEL_PUSHER_LONG_ARM_HALF_TARGET_POSITION);
        waitForMS(200);

        shortArm.setPosition(JEWEL_PUSHER_SHORT_ARM_INITIAL_POSITION);
        waitForMS(500);

        // long arm up
        //--------------------------------------
        longArm.setPosition(JEWEL_PUSHER_LONG_ARM_TELEOPS_POSITION);
    }

    /**
     * Do everything, extend, detect color, push, finally retract
     */
    public void performTasks() throws InterruptedException {

        // Task 1:
        //------------
        // move the long and short arm in the straight and down position
        // get ready to push (rotate short arm) the jewel
        //-------------------------------------------------------------
        extend();

        // Task 2:
        //--------------
        // rotate the short arm left or right to push the right color of jewel
        // out
        //------------------------------------------------------
        push();

        // Task 3:
        //---------------
        // move the long arm to upright position
        //----------------------------------------
        retract();
    }

    /**
     * push the opponent color of jewel off the position
     *
     * The color sensor is facing left Jewel, pay attention to the Alliance color
     *
     * if red alliance, push BLUE jewel
     * if blue alliance, push RED jewel
     */
    public void push() throws InterruptedException {

        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        Color jewelColor = getJewelColor();
        while(timer.time() < 3000 && jewelColor == UNKNOWN) {
            jewelColor = getJewelColor();
            waitForMS(100);
        }

        // red alliance, and red jewel detected,
        // then turn the right to push the blue
        Log.i("TechNova: " + this.getClass().getSimpleName(), "JewelColor:" + jewelColor);
        _telemetry.addData("JewelPusher", "JewelColor: " + jewelColor);

        if (this.alliance == AllianceColor.RED) {
            if(jewelColor == RED) {
                shortArm.setPosition(JEWEL_PUSHER_SHORT_ARM_RIGHT_PUSH_POSITION);
            } else if (jewelColor == BLUE) {
                shortArm.setPosition(JEWEL_PUSHER_SHORT_ARM_LEFT_PUSH_POSITION);
            }
        }
        else if (this.alliance == AllianceColor.BLUE) {
            if(jewelColor == BLUE) {
                shortArm.setPosition(JEWEL_PUSHER_SHORT_ARM_RIGHT_PUSH_POSITION);
            }
            else if (jewelColor == RED) {
                shortArm.setPosition(JEWEL_PUSHER_SHORT_ARM_LEFT_PUSH_POSITION);
            }
        } else {
            if (_telemetry != null) {
                _telemetry.addData("JewelPusher", timer.time() + " | No Jewel found !!!");
            }
        }

        waitForMS(1000);
    }

    /**
     * Helper method, use this only in Autonomous, which is in LinearOpMode
     *
     * @param milliSeconds
     */
    protected void waitForMS(long milliSeconds) throws InterruptedException {
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while (_autoOpMode.opModeIsActive() && timer.time() < milliSeconds) {
            Thread.sleep(100);
        }
    }

    /**
     * Detect Jewel color and log
     * @return
     */
    protected Color getJewelColor() {

        if(jewelColor != null) {
            String message = "Jewel Color: (" + jewelColor.red() + "|" + jewelColor.blue() + ") " + jewelColor.argb();
            Log.i("TechNova:" + this.getClass().getSimpleName(),  message);
            if(_telemetry != null) {
                _telemetry.addData("Jewel Pusher:", message);
            }
            if(jewelColor.red() > 10 && (jewelColor.red() - jewelColor.blue() >= 5) && jewelColor.argb() > 0) {
                return RED;
            } else if(jewelColor.blue() > 10 &&  (jewelColor.blue() - jewelColor.red() >= 5) && jewelColor.argb() > 0) {
                return BLUE;
            }
        }

        return UNKNOWN;
    }

    /**
     * Enumeration for Jewel Color
     */
    protected static enum Color {
        RED,
        BLUE,
        UNKNOWN
    }
}
