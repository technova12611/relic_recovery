package org.firstinspires.ftc.teamcode.TechNova2017;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import java.util.concurrent.TimeUnit;

/**
 * Provide the basic functions for robot to perform run to distance and turn to angle
 * and other sensor related control algorithms
 */
public abstract class RelicRecoveryAutoAbstract extends LinearOpMode {
    MecanumRobot robot = null;

    JewelPusher pusher = null;
    VuMarkVision vuMarkVision;
    RelicRecoveryVuMark vuMark;

    MovingAverage xAvgDistance = new MovingAverage(20);

    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    protected void initOpMode() throws InterruptedException {

        // initialize drive train and glyph lifts
        // this is shared for both Auto and TelOps
        //------------------------------------------------------

        telemetry.addData("Robot:", "Initializing ....");

        robot = new MecanumRobot(hardwareMap, telemetry, getAllianceColor());

        // VuMark is auto only, set up Vuforia and camera
        //-------------------------------------------------------
        vuMarkVision = new VuMarkVision(hardwareMap,telemetry);

        // Jewel Pusher is auto only
        //
        try {
            pusher = new JewelPusher(this, hardwareMap, telemetry, getAllianceColor(), true);
        }
        catch(InterruptedException ie) {
            throw ie;
        }
        catch(Exception e) {
            telemetry.addData("Jewel Pusher failed", e.getMessage());
            Log.e(this.getClass().getSimpleName(), "JewelPusher failed.", e);
        }

        // Activate vuMark
        //-----------------------------------------------------------
        vuMarkVision.activate();

        telemetry.addData("Robot:", "Initialized successfully, Ready to go ...");
        robot.logInfo(telemetry, "IMU Heading:", String.format("%.1f", robot.getHeadingAngle()));

        vuMark = vuMarkVision.detect(telemetry);

        AutoTransitioner.transitionOnStop(this, "Relic Recovery TeleOps (Linear)");

        ElapsedTime timer = new ElapsedTime();
        // detect VuMark pattern
        // try to get the VuMark for up to 5 seconds
        while(opModeIsActive() && !isStarted()
                && vuMark != null && vuMark == RelicRecoveryVuMark.UNKNOWN &&
                timer.time(TimeUnit.SECONDS) < 5) {
            vuMark = vuMarkVision.detect(telemetry);
            sleep(100);
        }
    }

    /**
     * These states neeed to be casted to the concrete class
     * depends on the game strategies
     */
    AutoState v_state;
    AutoState v_prev_state;

    protected void gotoNextState() {
        logStateInfo(v_state, "End");
        v_prev_state = v_state;
        v_state = v_state.next();
        timer.reset();
        robot.resetDriveMotors();
    }

    protected void gotoState(AutoState state) {
        logStateInfo(v_state, "End");
        v_prev_state = v_state;
        v_state = state;
        timer.reset();
        robot.resetDriveMotors();
    }

    protected void driveForwardInches(double inches, double power) throws InterruptedException {
        driveDirectionInches(0,inches, power);
    }

    protected void driveBackwardInches(double inches, double power) throws InterruptedException {
        driveDirectionInches(Math.PI,inches, power);
    }

    protected void driveLeftInches(double inches, double power) throws InterruptedException {
        driveDirectionInches(Math.PI*3/2.0,inches, power);
    }

    protected void driveRightInches(double inches, double power) throws InterruptedException {
        driveDirectionInches(Math.PI/2.0,inches, power);
    }

    protected void driveDirectionInches(double directionRadians, double inches, double power) throws InterruptedException {
        robot.setEncoderDrivePower(power);
        robot.encoderDriveInches(directionRadians, inches);
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && robot.driveMotorsBusy() && timer.time(TimeUnit.MILLISECONDS) < 5000) {
            robot.updateSensorTelemetry();
            telemetry.update();
            //robot.loop();
            idle();
        }
        robot.stopDriveMotors();
        robot.resetDriveMotorModes();
        robot.clearEncoderDrivePower();
    }

    private double angleDifference(double from, double to) {
        if (from < 0) from += 360;
        if (to < 0) to += 360;

        double diff = to - from;

        if (diff < -180) {
            diff += 360;
        } else if (diff > 180) {
            diff = - (360 - diff);
        }

        return diff;
    }

    /**
     * Using very basic control algorithm
     */
    private static final double SAFE_TURN_SPEED = 0.1;
    private static final double FAST_TURN_SPEED = 0.20;
    private static final double SUPER_FAST_TURN_SPEED = 0.3;
    private static final double FAST_TURN_THRESHOLD = 30.0;
    private static final double SUPER_FAST_TURN_THRESHOLD = 60.0;

    private static double speedForTurnDistance(double angle) {
        angle = Math.abs(angle);
        if (angle > SUPER_FAST_TURN_THRESHOLD) {
            return SUPER_FAST_TURN_SPEED;
        }
        if (angle > FAST_TURN_THRESHOLD) {
            return FAST_TURN_SPEED;
        }
        return SAFE_TURN_SPEED;
    }

    private static final int MAX_HEADING_SLOP = 1;

    /**
     * Handles the turning logics, using simple control, could be using PID
     *
     * @param degrees
     * @throws InterruptedException
     */
    private void turnToAngle(double degrees) throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();

        while(opModeIsActive() && timer.time(TimeUnit.MILLISECONDS) < 5000) {
            double diff = angleDifference(robot.getHeadingAngle(), degrees);
            if (MAX_HEADING_SLOP >= Math.abs(diff)) break;
            double speed = speedForTurnDistance(diff);
            robot.drive(0.0, 0.0, diff > 0 ? -speed : speed);
            idle();
        }
        robot.stopDriveMotors();
    }

    /**
     * Turning left is positive angle, turning right is negative angle
     *
     * @param degrees
     * @throws InterruptedException
     */
    protected void turn(double degrees) throws InterruptedException {
        turnToAngle(degrees);
    }

    /**
     * Log the sensor and motors data to logcat for analysis
     *
     * @param state
     * @param stage
     */
    void logStateInfo(AutoState state, String stage) {
        Log.i(this.getClass().getSimpleName(),
                String.format("%.1f",getRuntime()*1000.0)
                + " | " + timer.time(TimeUnit.MILLISECONDS)
                + " | State: " + state.toString()
                + " | " + stage
                + " | " + String.format("IMU: %.1f", robot.getHeadingAngle())
                + " | " + String.format("(x1,x2, y): %.1f, %.1f, %.1f", robot.getX1Distance(), robot.getX2Distance(), robot.getYDistance())
                + " | " + (vuMark != null? vuMark:"")
               );
    }

    void logInfo(String tag, String message) {
        Log.i(this.getClass().getSimpleName(), tag + " | " + message);
    }

    protected double getXDistance() {
        return getAllianceColor() == AllianceColor.RED? robot.getX1Distance(): robot.getX2Distance();
    }

    protected double measureXDistance(long elapseTime) {
        ElapsedTime timer = new ElapsedTime();
        double avg = 0.0;
        while(opModeIsActive() && timer.time(TimeUnit.MILLISECONDS) < elapseTime) {
            avg = xAvgDistance.next(robot.getX1Distance());
        }

        return avg;
    }

    // default is RED allaince
    // BLUE auto MUST override this
    //-----------------------------------------------
    public AllianceColor getAllianceColor() {
        return AllianceColor.RED;
    }

    protected double getRightColumnTargetDistanceInCM() {
        return 0.0;
    }

    protected double getCenterColumnTargetDistanceInCM() {
        return 0.0;
    }

    protected double geLeftColumnTargetDistanceInCM() {
        return 0.0;
    }
}