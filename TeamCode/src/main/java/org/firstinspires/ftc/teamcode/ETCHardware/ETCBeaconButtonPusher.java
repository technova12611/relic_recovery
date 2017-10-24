package org.firstinspires.ftc.teamcode.ETCHardware;

import android.hardware.camera2.params.TonemapCurve;
import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by FTC 9915 on 10/16/2016.
 */

public class ETCBeaconButtonPusher {

    private double delta_per_millisecond = 360/(1260*1400);
    private double position_per_inche = 1.0/(3.5*1.0*Math.PI);

    DigitalChannel pushSensor;

    private Servo pusherServo = null;

    private double initialPosition = 0.37;//2.0*position_per_inch;
    private double lastPosition = initialPosition;

    private double teleopInitialPosition = 0.37;

    private double increment = 0.003;

    private ElapsedTime pusherTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    private Boolean buttonPushed = null;

    public ETCBeaconButtonPusher(Servo pusher,
                                 DigitalChannel pushSensor) {
        this.pusherServo = pusher;
        this.pushSensor = pushSensor;
    }

    public void init() {

        this.pusherServo.setPosition(initialPosition);
        lastPosition = initialPosition;
    }

    public void teleop_init() {
        this.pusherServo.setPosition(teleopInitialPosition);
    }

    //extend
    public boolean pushButton() {

        if(pushSensor != null && !pushSensor.getState()) {
            if(buttonPushed == null) {
                pusherTimer.reset();
                buttonPushed = true;
            }

            if(buttonPushed && pusherTimer.time() > 325 ) {
                init();
                return true;
            }
        }

        double newPosition = Range.clip(lastPosition - increment, 0.05, initialPosition);
        pusherServo.setPosition(newPosition);
        lastPosition = newPosition;

        return false;
    }

    public void retract() {
        double newPosition = Range.clip(lastPosition + increment, 0.0, initialPosition);
        pusherServo.setPosition(newPosition);
        lastPosition = newPosition;
    }

    public void reset() {
        init();
        buttonPushed = null;
    }

    public double getPosition() {
        return pusherServo.getPosition();
    }
}
