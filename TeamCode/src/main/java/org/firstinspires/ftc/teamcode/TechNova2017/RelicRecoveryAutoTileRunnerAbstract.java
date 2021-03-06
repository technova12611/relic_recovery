package org.firstinspires.ftc.teamcode.TechNova2017;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import java.util.concurrent.TimeUnit;

import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.DISTANCE_SENSOR_UPRIGHT_POSITION;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.DISTANCE_SENSOR_UPRIGHT_POSITION_2;

/**
 * Provide the basic functions for robot to perform run to distance and turn to angle
 * and other sensor related control algorithms
 */
public abstract class RelicRecoveryAutoTileRunnerAbstract extends LinearOpMode {
    TileRunnerRobot robot = null;

    public JewelPusher pusher = null;
    VuMarkVision vuMarkVision;
    RelicRecoveryVuMark vuMark;

    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    protected void initOpMode() throws InterruptedException {

        // initialize drive train and glyph lifts
        // this is shared for both Auto and TelOps
        //------------------------------------------------------

        telemetry.addData("Robot:", "Initializing ....");

        robot = new TileRunnerRobot(this,hardwareMap, telemetry, getAllianceColor());

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

        AutoTransitioner.transitionOnStop(this, "Tile Runner (New) TeleOps");

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
        //robot.resetDriveMotors();
    }

    protected void gotoState(AutoState state) {
        logStateInfo(v_state, "End");
        v_prev_state = v_state;
        v_state = state;
        timer.reset();
        robot.resetDriveMotors();
    }

    protected void sleepInAuto(long milSec) {
        ElapsedTime sleepTimer = new ElapsedTime();
        while(opModeIsActive() && sleepTimer.milliseconds() < milSec)  {
            sleep(50);
        }
    }

    protected void driveForwardInches(double inches, double power, double timeout) throws InterruptedException {
        logInfo(" Drive forward:",
                String.format("%.2f, %.2f, %.2f", inches, power, timeout));
        driveDirectionInches(Math.PI,inches, power, timeout);
    }

    protected void driveForwardInchesToColumn(double inches, double power, double timeout) throws InterruptedException {
        logInfo(" Drive forward to Column:",
                String.format("%.2f, %.2f, %.2f, %.2f", inches, power, timeout, robot.getColDistance()));
        driveDirectionInches(Math.PI,inches, power, timeout, true);
    }

    protected void driveBackwardInches(double inches, double power, double timeout) throws InterruptedException {
        logInfo(" Drive backward:",
                String.format("%.2f, %.2f, %.2f", inches, power, timeout));
        driveDirectionInches(0,inches, power, timeout);
    }

    protected boolean driveBackwardInchesToColumn(double inches, double power, double timeout) throws InterruptedException {
        logInfo(" Drive backward to Column:",
                String.format("%.2f, %.2f, %.2f, %.2f, ", inches, power, timeout, robot.getColDistance()) + robot.columnDetected());
        return driveDirectionInches(0,inches, power, timeout, true);
    }

    protected void driveLeftInches(double inches, double power, double timeout) throws InterruptedException {
        logInfo(" Strafe left:",
                String.format("%.2f, %.2f, %.2f", inches, power, timeout));
        driveDirectionInches(Math.PI*3/2.0,inches, power, timeout);
    }

    protected void driveRightInches(double inches, double power, double timeout) throws InterruptedException {
        logInfo(" Strafe right:", String.format("%.2f, %.2f, %.2f", inches, power, timeout));
        driveDirectionInches(Math.PI/2.0,inches, power, timeout);
    }

    protected void driveDirectionInches(double directionRadians, double inches, double power) throws InterruptedException {
        driveDirectionInches(directionRadians,inches, power, 5.0);
    }

    protected void driveDirectionInches(double directionRadians, double inches, double power, double timeout) throws InterruptedException {
        driveDirectionInches(directionRadians,inches, power, timeout, false);
    }

    protected boolean driveDirectionInches(double directionRadians, double inches, double power, double timeout, boolean useRangerSensor) throws InterruptedException {
        boolean columnTouched = false;
        robot.setEncoderDrivePower(power);
        robot.encoderDriveInches(directionRadians, inches);
        ElapsedTime timer = new ElapsedTime();

        double colDistance = 0.0;
        if(useRangerSensor) {
            colDistance = robot.getColDistance();
            logInfo("*** Sensor drive (pre) - Column detected", robot.columnDetected() + " | "
                         + String.format("%.2f", colDistance));
        }

        while (opModeIsActive() && robot.driveMotorsBusy() && timer.seconds() < timeout &&
                (!useRangerSensor || robot.columnDetected() == null || !robot.columnDetected()))
        {
            if(useRangerSensor && robot.isColumnTouched() != null) {
                if(robot.isColumnTouched()) {
                    logInfo("*** Column touched", robot.isColumnTouched() + "");
                    columnTouched = true;
                    break;
                }
            }

            if(useRangerSensor) {
                colDistance = robot.getColDistance();
                if (colDistance < 5.5) {
                    // break;
                    logInfo("*** Column distance", String.format("%.2f", colDistance));
                }
            }
            //robot.updateSensorTelemetry();
            //telemetry.update();
            //robot.loop();
            idle();
        }

        robot.updateSensorTelemetry();
        telemetry.update();

        robot.stopDriveMotors();
        robot.resetDriveMotorModes();
        robot.clearEncoderDrivePower();

        logInfo("Encoder drive", String.format("%.3f (s)", timer.seconds()) + " | " + robot.columnDetected());

        return columnTouched;
    }

    protected void driveDirectionInchesUntilPlyphHit(double directionRadians, double inches, double power) throws InterruptedException {
        robot.setEncoderDrivePower(power);
        robot.encoderDriveInches(directionRadians, inches);
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && robot.driveMotorsBusy() && timer.time(TimeUnit.MILLISECONDS) < 5000 ) {
            //robot.updateSensorTelemetry();
            //telemetry.update();
            //robot.loop();
            idle();
        }

        robot.updateSensorTelemetry();
        telemetry.update();

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
    private static final double VERY_SAFE_TURN_SPEED = 0.10;
    private static final double SAFE_TURN_SPEED = 0.15;
    private static final double FAST_TURN_SPEED = 0.25;
    private static final double SUPER_FAST_TURN_SPEED = 0.35;
    private static final double FAST_TURN_THRESHOLD = 45.0;
    private static final double FINAL_TURN_THRESHOLD = 20.0;
    private static final double SUPER_FAST_TURN_THRESHOLD = 60.0;

    private double speedForTurnDistance(double angle) {
        if (angle > SUPER_FAST_TURN_THRESHOLD) {
            return SUPER_FAST_TURN_SPEED;
        }
        if (angle > FAST_TURN_THRESHOLD) {
            return FAST_TURN_SPEED;
        }

        if(angle> FINAL_TURN_THRESHOLD) {
            return SAFE_TURN_SPEED;
        }

        return VERY_SAFE_TURN_SPEED;
    }

    private static final double MAX_HEADING_SLOP = 1.5;

    /**
     * Handles the turning logics, using simple control, could be using PID
     *
     * @param degrees
     * @throws InterruptedException
     */
    private void turnToAngle(double degrees) throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();

        double headingAngle = 0.0;
        Double initHeadingAngle = null;
        while(opModeIsActive() && timer.time(TimeUnit.MILLISECONDS) < 5000) {
            headingAngle = robot.getHeadingAngle();
            if(initHeadingAngle == null) {
                initHeadingAngle = headingAngle;
            }
            double diff = angleDifference(headingAngle, degrees);
            if (MAX_HEADING_SLOP >= Math.abs(diff)) break;

            double speed = speedForTurnDistance(Math.abs(diff));
            //logInfo("** TurnToAngle: ", String.format("%.1f %.1f %.2f", headingAngle, diff, speed));
            robot.drive(0.0, 0.0, diff > 0 ? speed : -speed);
            idle();
        }
        robot.stopDriveMotors();

        logInfo(" -- Angle Reached -- :", String.format("(%.1f, %.1f)", initHeadingAngle, headingAngle) + " | " +  timer.time(TimeUnit.MILLISECONDS));
    }

    protected void turnToAngle(double target, double speed) throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();

        double headingAngle = 0.0;
        Double initHeadingAngle = null;
        while(opModeIsActive() && timer.time(TimeUnit.MILLISECONDS) < 5000) {
            headingAngle = robot.getHeadingAngle();
            if(initHeadingAngle == null) {
                initHeadingAngle = headingAngle;
            }
            double diff = angleDifference(headingAngle, target);
            if (MAX_HEADING_SLOP >= Math.abs(diff)) break;

            //logInfo("TurnToAngle 2: ", String.format("%.1f %.1f %.2f", headingAngle, diff, speed));
            robot.drive(0.0, 0.0, diff > 0 ? speed : -speed);
            idle();
        }
        robot.stopDriveMotors();

        logInfo(" -- Angle Reached -- :",
                String.format("(%.1f, %.1f)", initHeadingAngle, headingAngle) + " | " + timer.time(TimeUnit.MILLISECONDS));
    }

    private String getRunTimeString() {
        return String.format(" Runtime: %.2f ", getRuntime());
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
        try {
            Log.i(this.getClass().getSimpleName(), "TechNova: " +
                    getAllianceColor()
                            + " | State: " + String.format("%28s", state.toString())
                            + " | " + String.format("%6s", stage)
                            + " | " + String.format("%6.1f", getRuntime() * 1000.0)
                            + " | " + String.format("%5d", timer.time(TimeUnit.MILLISECONDS))
                            + " | " + String.format("IMU: %.1f", robot.getHeadingAngle())
                            + " | " + String.format("(x1,x2): %3.1f, %3.1f", robot.getX1Distance(), robot.getColDistance()) + ", " + robot.columnDetected()
            //                + " | " + "Glyph count:" + String.format("%5d", robot.getGlyphLiftPosition())
                            + " | " + (vuMark != null ? vuMark : "")
                            + " | " + String.format("Battery: %3.2f", robot.getBatteryVoltage())
            );
        }catch(Exception e) {
            Log.i(this.getClass().getSimpleName(), "Failed in LogStateInfo: " + e.getMessage());
        }
    }

    void logInfo(String tag, String message) {
        Log.i("TechNova: " + this.getClass().getSimpleName(), tag + " | " + message + " | " + getRunTimeString());
    }

    void logInfo(String message) {
        Log.i("TechNova: " + this.getClass().getSimpleName(), message + " | " + getRunTimeString());
    }

    protected double getXDistance() {
        return getAllianceColor() == AllianceColor.RED? robot.getX1Distance(): robot.getColDistance();
    }

    protected double measureColDistance(long elapseTime) {
        ElapsedTime timer = new ElapsedTime();
        double avg = 0.0;
        MovingAverage colAvgDistance = new MovingAverage(10);
        while(opModeIsActive() && timer.time(TimeUnit.MILLISECONDS) < elapseTime) {
            double distance = robot.getColDistance();
            logInfo("        * Raw Col Range Sensor (in): " + String.format("%.2f", distance));
            if(distance > 0.05 && distance < 8.0 ) {
                avg = colAvgDistance.next(distance);
                logInfo("        * Moving Avg. Range Sensor (in): " +
                        String.format("%.2f", avg) + " | " + timer.time(TimeUnit.MILLISECONDS));
            } else {
                logInfo("        * Raw Col Range Sensor out of range. ( < 0.5 or > 8.0)");
            }
            sleep(50);
        }

        logInfo("    Avg. Range Col Sensor (in): " + String.format("%.2f", avg));

        return avg;
    }

    protected void placeGlyphIntoColumn(double motorSpeed) throws InterruptedException {
        placeGlyphIntoColumn(motorSpeed, true);
    }

    protected void placeGlyphIntoColumn(double motorSpeed, boolean makeTurn) throws InterruptedException {

        robot.pushGlyph();
        boolean columnDetected = robot.columnDetected();
        logInfo("** Place glyph into column ** ");

        logInfo(" --- Align robot to the cryptobox ( Col: " +
                columnDetected + ", touched: " + robot.isColumnTouched() + ") --- ");
        boolean aligned = false;
        double startTime = getRuntime();

        if(getRuntime() < 28.55) {
            aligned = alignCryptoBoxInAuto(5.0);
        }

        logInfo(" --- Flip Glyph Tray (" + robot.columnDetected() + ") --- " );
        robot.dumpGlyphsFromTray();
        sleepInAuto(300);

        logInfo(" --- More backward to let glyph fall on the floor --- ");

        if(getRuntime() > 28.75) {
            driveForwardInches(4.5, 0.60, 1.0);
        } else if(getRuntime() > 28.00){
            if(!aligned) {
                driveBackwardInches(2.0, 0.5, 1.0);
            }
            driveForwardInches(5.0, 0.6, 1.0);
        } else {
            driveForwardInches(3.5, motorSpeed, 1.0);
            if (!aligned) {
                sleepInAuto(200);
                driveForwardInches(3.5, motorSpeed, 1.0);
            }
        }

        if(getRuntime() < 28.0) {

            // move forward to push the glyph into the box
            //-------------------------------------------------
            ElapsedTime watcher = new ElapsedTime();
            if(!aligned) {
                logInfo(" --- Drive forward to push --- ");
                driveBackwardInches(7.5, motorSpeed, 1.25);
            }

            logInfo(" --- Place Glyph into column (ms): " +
                    watcher.time(TimeUnit.MILLISECONDS) + " | " + vuMark);

            // need to push again
            if (watcher.seconds() > 1.8) {
                logInfo(" --- Missed the column, push again --- ");
                //driveForwardInches(2.0, motorSpeed, 2.0);
                //driveBackwardInches(4.0, motorSpeed, 2.0);
            }
            // move backward to separate robot from glyph
            //----------------------------------------------
            if(!aligned) {
                logInfo(" --- Drive backward to finish --- ");
                driveForwardInches(8.5, 0.5, 2.0);
            }

            if(makeTurn) {
                logInfo(" --- Turn to 90 degree --- ");
                turn(-89.0);
            }
        }

        robot.holdPusher();
        robot.resetGlyphTray();
        robot.uprightDistanceSensorArmServo();

        // close the glyphBlocker
        //----------------------------------------
        robot.closeGlyphBlocker();
        logInfo(" --- Finished glyphs --- " + String.format("%.2f", (getRuntime()-startTime)));
    }

    public boolean alignCryptoBoxInAuto(double timeOutInSeconds) throws InterruptedException {

        boolean aligned = false;

        double distance = robot.getColDistance();

        logInfo(" --- Alignment: Init col distance: " + String.format("%.2f", distance));
        //robot.extendDistanceSensorArmServo();

        if(distance > 200.0) {
            logInfo("Couldn't read range sensor data.");
            ElapsedTime timer = new ElapsedTime();
            while (opModeIsActive() && timer.milliseconds() < 250.0) {
                distance = robot.getColDistance();
            }

            if(distance > 200.0) {
                return false;
            }
        }

        ElapsedTime timer1 = new ElapsedTime();
        int count = 0;
        double runtime = 27.5;
        while(opModeIsActive() && timer1.seconds() < timeOutInSeconds && getRuntime() < runtime) {
            //distance = measureColDistance(150);
            distance = robot.getColDistance();
            logInfo("    Col distance (in): " + String.format("%.2f", distance) + " | " + robot.columnDetected());

            // too far from cryptobox, move in by 2 inches
            if(distance > 6.5 && distance < 15.0 && !robot.columnDetected()) {
                //driveBackwardInchesToColumn(3.0, 0.15, 1.0);
                distance = robot.getColDistance();
                logInfo("     *** Adjusted distance from the column (in): " + String.format("%.2f", distance) + " | " + robot.columnDetected());
            }

            // measurement as inches
            // need to test and tweak this to make it accurate
            //---------------------------------------------------
            //
            if (distance > 0.0 && distance < 15.0) {
                double desiredDistance = 3.00;
                double delta = distance - desiredDistance;

                logInfo("    Delta from the column (in): " + String.format("%.2f", delta));

                if (delta > 0.4 ) {
                    driveRightInches(delta*1.5, 0.35, 1.5);
                    if(robot.getColDistance() == 2.0) {
                        break;
                    }
                } else if (delta < -0.4) {
                    driveLeftInches(-delta*1.5, 0.35, 1.5);

                    if(robot.getColDistance() == 2.0) {
                        break;
                    }
                } else {
                    aligned = true;
                    logInfo("**** Aligned correctly.");
                    break;
                }

                if(timer1.milliseconds() > 350) {
                    robot.pushGlyph();
                }
                else if(timer1.milliseconds() > 200) {
                    robot.holdPusher();
                }

                runtime = 28.5;
            } else {
                logInfo("!!!! Range Sensor out of range. | " + String.format("%.2f", distance));

                if(distance > 200) {
                    ElapsedTime timerS = new ElapsedTime();
                    int retries = 0;
                    while (opModeIsActive() && timerS.milliseconds() < 500) {
                        distance = robot.getColDistance();
                        logInfo("!!!! Out of range, retry #" + ++retries + " | " + String.format("%.2f", distance));
                        if (distance < 200.0) {
                            break;
                        }
                    }
                }

                if(distance > 20.0) {
                    break;
                }
            }

            logInfo("    " + (++count) + " Distance from the column (in): " + String.format("%.2f", robot.getColDistance()));
        }

        logInfo("    Final Distance from the column (in): " + String.format("%.2f", robot.getColDistance()));

        robot.setServoPosition(robot.distSensorServo, DISTANCE_SENSOR_UPRIGHT_POSITION);

        return aligned;
    }

    protected boolean isGlyphStucked() throws InterruptedException {
        sleepInAuto(100);
        int previousIntakeCount = robot.intakeRight.getCurrentPosition();
        sleepInAuto(150);
        robot.holdPusher();

        logInfo("    Intake encoder: ( " + previousIntakeCount + "," + robot.intakeRight.getCurrentPosition() + ")");

        boolean glyphStucked = false;
        if(Math.abs(previousIntakeCount - robot.intakeRight.getCurrentPosition()) < 50) {
            robot.reverseGlyph();
            sleepInAuto(2000);
            robot.collectGlyph();
            glyphStucked = true;

            driveForwardInches(7.0, 0.35, 1.0);
            sleepInAuto(100);
            driveBackwardInches(7.0, 0.35, 1.0);

            robot.pushGlyph();
        }
        return glyphStucked;
    }

    // default is RED allaince
    // BLUE auto MUST override this
    //-----------------------------------------------
    public AllianceColor getAllianceColor() {
        return AllianceColor.RED;
    }

    protected boolean pickupMoreGlyphs() {
        return false;
    }

    protected boolean dumpMoreGlyphs() { return false; }

}