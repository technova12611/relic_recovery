package org.firstinspires.ftc.teamcode.TechNova2017;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 *  One motor for lifting glyphs, two servos for grabbing glyphs
 *    1. should have one method to lift()
 *    2. should have one method to grab glyph, servo.setPosition
 *    3. should have one method to release glyph, servo.setPosition
 */
public class GlyphLift {

    Servo upperLeftGripper, upperRightGripper, lowerLeftGripper, lowerRightGripper;
    DcMotor glyphLift;

    /**
     * initializa motor and two servos
     *
     * @param hardwareMap
     */
    public void init(HardwareMap hardwareMap) {
        upperLeftGripper = hardwareMap.servo.get("upperLeftGripper");
        upperRightGripper = hardwareMap.servo.get("upperRightGripper");

        lowerLeftGripper = hardwareMap.servo.get("lowerLeftGripper");
        lowerRightGripper = hardwareMap.servo.get("lowerRightGripper");

        glyphLift = hardwareMap.dcMotor.get("glyphLift");
        glyphLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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