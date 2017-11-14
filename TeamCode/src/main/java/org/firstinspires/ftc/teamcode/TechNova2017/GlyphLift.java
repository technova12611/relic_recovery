package org.firstinspires.ftc.teamcode.TechNova2017;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.GLYPH_TOP_HOLDER_INITIAL_POSITION;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.LOWER_LEFT_GLYPH_ARM_INITIAL_POSITION;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.LOWER_RIGHT_GLYPH_ARM_INITIAL_POSITION;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.UPPER_LEFT_GLYPH_ARM_INITIAL_POSITION;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.UPPER_RIGHT_GLYPH_ARM_INITIAL_POSITION;

/**
 *  One motor for lifting glyphs, two servos for grabbing glyphs
 *    1. should have one method to lift()
 *    2. should have one method to grab glyph, servo.setPosition
 *    3. should have one method to release glyph, servo.setPosition
 */
public class GlyphLift {

    Servo upperLeftGripper, upperRightGripper, lowerLeftGripper, lowerRightGripper, glyphHolder;
    DcMotor glyphLift;

    /**
     * initializa motor and two servos
     *
     * @param hardwareMap
     */
    public void init(HardwareMap hardwareMap) {
        upperLeftGripper.setPosition(UPPER_LEFT_GLYPH_ARM_INITIAL_POSITION);
        upperRightGripper.setPosition(UPPER_RIGHT_GLYPH_ARM_INITIAL_POSITION);

        lowerLeftGripper = hardwareMap.servo.get("lowerLeftGripper");
        lowerRightGripper = hardwareMap.servo.get("lowerRightGripper");

        lowerLeftGripper.setPosition(LOWER_LEFT_GLYPH_ARM_INITIAL_POSITION);
        lowerRightGripper.setPosition(LOWER_RIGHT_GLYPH_ARM_INITIAL_POSITION);

        try {
            glyphHolder = hardwareMap.servo.get("glyphHolder");
            glyphHolder.setPosition(GLYPH_TOP_HOLDER_INITIAL_POSITION);
        } catch(Exception e) {
            Log.e(this.getClass().getSimpleName(), e.getMessage());
        }

        glyphLift = hardwareMap.dcMotor.get("glyphLift");
        glyphLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        glyphLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * set power to the glyph motor
     * may use encoder to set lower limit etc.
     *
     * @param power
     */
    public void lift(double power) {

    }

    /**
     * use servo to grab glyph
     *   set servo to the grab position
     */
    public void grab() {

    }

    /**
     * use servo to release glyph, just enough to let the glyph(s) drop
     *     set servo to the release position
     */
    public void release() {

    }

    /**
     * Open the arm wider to let the glyph coming in
     */
    public void open() {

    }
}