package org.firstinspires.ftc.teamcode.TechNova2017;

import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.TechNova2017.AllianceColor.*;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.*;

/**
 *  Use one motor to extend or collapse the slider mechanism
 *  Use one or two servo(s) for grabbing the relic
 *
 *  1. The slider should be able to extend out and collapse back in motor.setPower()
 *  2. Grabber servo should be able to grab and release, servo.setPosition()
 *  3. May have arm to raise or lwoer the Relic, servo.setPosition()
 */
public class RelicSlider {

    /**
     * initialize motor and servos
     *
     * @param hardwarMap
     */
    public void init(HardwareMap hardwarMap) {

    }

    /**
     * this can be used to extend or collapse
     * likely the power will be mapped to joystick
     */
    public void slide(double power) {

    }

    /**
     * set the servo to a known position that grabs the Relic
     */
    public void grabRelic() {

    }

    /**
     * set the servo to a known position that releaae the Relic
     */
    public void releaseRelic() {

    }
}
