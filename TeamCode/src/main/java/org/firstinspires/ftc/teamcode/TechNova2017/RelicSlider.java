package org.firstinspires.ftc.teamcode.TechNova2017;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.RELIC_CLAWHOLDER_INITIAL_POSITION_2;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.RELIC_CLAW_INITIAL_POSITION;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.RELIC_ELBOW_INITIAL_POSITION;

/**
 *  Use one motor to extend or collapse the slider mechanism
 *  Use one or two servo(s) for grabbing the relic
 *
 *  1. The slider should be able to extend out and collapse back in motor.setPower()
 *  2. Grabber servo should be able to grab and release, servo.setPosition()
 *  3. May have arm to raise or lwoer the Relic, servo.setPosition()
 */
public class RelicSlider {

    private DcMotor relicSlider;
    private Servo relicClaw, relicElbow, relicClawholder;
    /**
     * initialize motor and servos
     *
     * @param hardwareMap
     */
    public void init(HardwareMap hardwareMap) {
        relicSlider = hardwareMap.dcMotor.get("relicSlider");
        relicSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        relicSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        try {
            relicClaw = hardwareMap.servo.get("relicClaw");
            relicClaw.setPosition(RELIC_CLAW_INITIAL_POSITION);
            relicElbow = hardwareMap.servo.get("relicElbow");
            relicElbow.setPosition(RELIC_ELBOW_INITIAL_POSITION);
            relicClawholder = hardwareMap.servo.get("relicClawholder");
            relicClawholder.setPosition(RELIC_CLAWHOLDER_INITIAL_POSITION_2);
        } catch(Exception e) {
            Log.e(this.getClass().getSimpleName(), "Init relic failed: " + e.getMessage());
        }

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
