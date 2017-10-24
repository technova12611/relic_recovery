package org.firstinspires.ftc.teamcode.ETC2016OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ETCHardware.ETC2016Robot;

/**
 * Created by Robotics 5 on 10/23/2016.
 */

@TeleOp(name="RegularWheelRobot Test", group = "9915")
@Disabled
public abstract class ETCRobotTest extends ETC2016AutoBaseOpMode {

    private boolean buttonPressed = false;
    private Boolean whiteTapeDetected = null;

    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    protected boolean initGyro() {
        return false;
    }

    @Override
    public void start() {
        timer.reset();
        robot.stopperServo.setPosition(0.1);
    }
    @Override
    public void loop() {

        ETC2016Robot.BeaconColor bc = robot.getBeaconColorSensor();
        if(bc == ETC2016Robot.BeaconColor.BAD_SENSOR ) {
            telemetry.addData("0a", "No Color Sensor...");
            return;
        }

        if( bc == ETC2016Robot.BeaconColor.RED ) {
            if(timer.time() < 5000) {
                robot.buttonPusher.pushButton();
                timer.reset();
            } else {
                buttonPressed = true;
                robot.buttonPusher.reset();
            }
        }
        else if(bc == ETC2016Robot.BeaconColor.BLUE ){
            robot.buttonPusher.retract();
            timer.reset();
        }

        if(buttonPressed && timer.time() < 5000) {
            robot.buttonPusher.retract();
        }

        if(robot.detectWhiteTape()) {
            if(whiteTapeDetected == null) {
                timer.reset();
                whiteTapeDetected = true;
            }
            if(timer.time() < 2000) {
                robot.stopperServo.setPosition(0.0);
            }

            else if(timer.time() >= 2000 && timer.time() < 5000) {
                robot.stopperServo.setPosition(0.35);
            }
            else if(timer.time() >= 5000 && timer.time() < 8000){
                robot.stopperServo.setPosition(0.75);
            } else {
                robot.stopperServo.setPosition(1.0);
            }
        }

        if(gamepad2.a) {
            timer.reset();
            buttonPressed = false;
            whiteTapeDetected = false;
            robot.buttonPusher.reset();
            robot.buttonPusher.reset();
        }
        updateSensorInfoInTelemetry();
    }
}
