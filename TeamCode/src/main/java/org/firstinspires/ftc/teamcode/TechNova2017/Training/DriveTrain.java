package org.firstinspires.ftc.teamcode.TechNova2017.Training;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.TechNova2017.BoschGyroSensor;

/**
 * Class for two motor 4-6 wheels drive train
 * with Gyro sensor to make accurate turns
 */
public class DriveTrain {
    protected DcMotor leftMotor;
    protected DcMotor rightMotor;

    protected BoschGyroSensor gyroSensor;

    public void init(HardwareMap hardwareMap) {

        Log.i(this.getClass().getSimpleName(), "-- Initilizing motors --");
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Log.i(this.getClass().getSimpleName(), "-- Completed motors initilization --");

        gyroSensor = new BoschGyroSensor();
        gyroSensor.init(hardwareMap);
    }

    /**
     * Using the Motor power mode (RUN_WITHOUT_ENCODER)
     * No Motor PID
     *
     * @param leftPower
     * @param rightPower
     */
    public void setPower(double leftPower, double rightPower) {
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
    }

    /**
     * Using the Motor Speed Mode, RUN_USING_ENCODER
     * Using Motor PID to keep the speed
     *
     * @param leftSpeed
     * @param rightSpeed
     */
    public void setSpeed(double leftSpeed, double rightSpeed) {
        this.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.leftMotor.setPower(leftSpeed);
        this.rightMotor.setPower(rightSpeed);
   }

    /**
     * Using Motor Position mode, RUN_TO_POSITION
     * Using Motor PID to reach targeted position
     *
     * @param maxSpeed
     * @param targetPosition
     * @return
     */
    public boolean setRunToPosition(double maxSpeed, int targetPosition) {
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setTargetPosition(targetPosition);
        rightMotor.setTargetPosition(targetPosition);

        double speed = Range.clip(Math.abs(maxSpeed), 0.0, 1.0);

        leftMotor.setPower(speed);
        rightMotor.setPower(speed);

        if(!leftMotor.isBusy() && !rightMotor.isBusy()) {
            stop();
            return true;
        }

        return false;
    }

    protected boolean pidTurn(double targetAngle, double speed, double kp) {
        double error;
        double power;
        double modifier = speed<0?-1:1;

        error = getError(targetAngle);
        double tolerence = Math.abs(speed) < 0.35?1.0:3.0;
        if(Math.abs(error) <=tolerence) {
            setPower(0.0, 0.0);
            return true;
        }

        power = kp * error;

        if(power <0) {
            power = Range.clip(power, -Math.abs(speed), -0.035);
        } else {
            power = Range.clip(power, 0.035, Math.abs(speed));
        }

        setPower(-modifier*power, modifier*power);

        return false;
    }

    private double getError(double targetAngle) {
        double robotError;
        // calculate error in -179 to +180 range  (
        robotError = targetAngle - getHeadingAngle();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public void stop() {
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public double getHeadingAngle() {
        return gyroSensor.getHeadingAngle();
    }
}
