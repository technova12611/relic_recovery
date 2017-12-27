package org.firstinspires.ftc.teamcode.TechNova2017;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.ENCODER_DRIVE_POWER;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.GLYPH_LIFT_PICKUP_GLYPH_IN_AUTO_POSITION;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.GLYPH_LIFT_STOPPER_CLOSE_POSITION;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.GLYPH_LIFT_STOPPER_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.GLYPH_TOP_HOLDER_CLOSE_POSITION;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.GLYPH_TOP_HOLDER_INITIAL_POSITION;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.GLYPH_TOP_HOLDER_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.JEWEL_PUSHER_LONG_ARM_TELEOPS_POSITION;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.LOWER_LEFT_GLYPH_ARM_CLOSE_POSITION;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.LOWER_LEFT_GLYPH_ARM_INITIAL_POSITION;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.LOWER_LEFT_GLYPH_ARM_MEDIUM_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.LOWER_LEFT_GLYPH_ARM_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.LOWER_LEFT_GLYPH_ARM_WIDE_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.LOWER_RIGHT_GLYPH_ARM_CLOSE_POSITION;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.LOWER_RIGHT_GLYPH_ARM_INITIAL_POSITION;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.LOWER_RIGHT_GLYPH_ARM_MEDIUM_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.LOWER_RIGHT_GLYPH_ARM_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.LOWER_RIGHT_GLYPH_ARM_WIDE_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.RELIC_CLAWHOLDER_INITIAL_POSITION;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.RELIC_CLAWHOLDER_RELEASE_POSITION;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.RELIC_CLAW_CLOSE_POSITION;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.RELIC_CLAW_INITIAL_POSITION;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.RELIC_CLAW_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.RELIC_ELBOW_INITIAL_POSITION;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.RELIC_ELBOW_RELEASE_POSITION;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.RELIC_ELBOW_UP_POSITION;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.UPPER_LEFT_GLYPH_ARM_CLOSE_POSITION;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.UPPER_LEFT_GLYPH_ARM_INITIAL_POSITION;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.UPPER_LEFT_GLYPH_ARM_MEDIUM_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.UPPER_LEFT_GLYPH_ARM_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.UPPER_LEFT_GLYPH_ARM_WIDE_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.UPPER_RIGHT_GLYPH_ARM_CLOSE_POSITION;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.UPPER_RIGHT_GLYPH_ARM_INITIAL_POSITION;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.UPPER_RIGHT_GLYPH_ARM_MEDIUM_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.UPPER_RIGHT_GLYPH_ARM_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.UPPER_RIGHT_GLYPH_ARM_WIDE_OPEN_POSITION;

/**
 * 2017 Relic Recovery Game Robot
 *   A Mecanum drive robot (4 motors)
 *
 *   1. Glyph Lift - Motor
 *   2. Glyph Grippers (left & right Servos)
 *   3. Relic Lifts (Vertical & Horizontal Motors)
 *   4. Relic Grabber  (Servo)
 *
 *   5. Jewel Pusher (2 servos and one color)
 */
public class MecanumRobot {
    private DcMotor lf, lr, rf, rr, led, glyphLift, relicSlider;
    private Servo upperLeftGripper, upperRightGripper,lowerLeftGripper, lowerRightGripper,
            glyphHolder, glyphLiftStopper,
            longArm,
            relicClaw, relicElbow, relicClawholder;

    private Telemetry telemetry;
    private VoltageSensor voltageSensor;

    private boolean isBlueLedOn = false;
    private boolean isGreenLedOn = false;

    private BNO055IMU imu;
    private Orientation angles;

    private ModernRoboticsI2cRangeSensor x1RangeSensor;
    private ModernRoboticsI2cRangeSensor x2RangeSensor;
    private ModernRoboticsI2cRangeSensor yRangeSensor;

    private ColorSensor glyphColor;
    private DistanceSensor glyphDistance;

    private double headingOffset = 0.0;

    private double prevX1Distance = 0.0;
    private double prevX2Distance = 0.0;
    private double prevYDistance = 0.0;

    private LinearOpMode linearOpMode;

    private AnalogInput rangeSensor;
    double maxRangeVol = 0.0;

    double previousDriveAvgEncoder = 0;

    MovingAverage driveEncodersMovingAvg = new MovingAverage(3);

    // Encoder Driving
    // Assuming 4" wheels
    private static final double TICKS_PER_INCH = 1120 * (24./32.) / (Math.PI * 4.0);
    private static final double TICKS_PER_CM = TICKS_PER_INCH / 2.54;

    private double encoder_drive_power = ENCODER_DRIVE_POWER;

    /**
     * Wheel motors MUST be named: lf, rf, lr, rr
     *
     * Glyph Lift Motor MUST be named: glyphLift
     *
     * Glyph Gripper MUST be named: leftGripper, rightGripper
     *
     * @param hardwareMap
     * @param _telemetry
     */
    public MecanumRobot(LinearOpMode opMode, HardwareMap hardwareMap, Telemetry _telemetry, AllianceColor allianceColor) {
        this.linearOpMode = opMode;
        this.telemetry = _telemetry;
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        try {
            relicClawholder = hardwareMap.servo.get("relicClawholder");
            relicClawholder.setPosition(RELIC_CLAWHOLDER_INITIAL_POSITION);
        } catch(Exception e) {
            logInfo(this.telemetry, "Init relic claw holder failed", e.getMessage());
        }

        initMotors(hardwareMap);

        if(allianceColor != null) {
            initGyro(hardwareMap);
            initRangeSensor(hardwareMap, allianceColor);

            try {
                rangeSensor = hardwareMap.analogInput.get("range");
                maxRangeVol= rangeSensor.getMaxVoltage();
            }catch(Exception e) {
            }

        } else {
            longArm = hardwareMap.servo.get("longArm");
            // make sure long arm is in the up right position
            longArm.setPosition(JEWEL_PUSHER_LONG_ARM_TELEOPS_POSITION);

            glyphColor = hardwareMap.get(ColorSensor.class, "glyphColorDistance");

            // get a reference to the distance sensor that shares the same name.
            glyphDistance = hardwareMap.get(DistanceSensor.class, "glyphColorDistance");
        }

        try {
            glyphHolder = hardwareMap.servo.get("glyphHolder");
            glyphHolder.setPosition(GLYPH_TOP_HOLDER_INITIAL_POSITION);

            glyphLiftStopper = hardwareMap.servo.get("glyphLiftStopper");
            glyphLiftStopper.setPosition(GLYPH_LIFT_STOPPER_OPEN_POSITION);
        } catch(Exception e) {
            logInfo(this.telemetry, "Init glyph holder", e.getMessage());
        }

        upperLeftGripper = hardwareMap.servo.get("upperLeftGripper");
        upperRightGripper = hardwareMap.servo.get("upperRightGripper");

        lowerLeftGripper = hardwareMap.servo.get("lowerLeftGripper");
        lowerRightGripper = hardwareMap.servo.get("lowerRightGripper");

        if(allianceColor != null) {
            upperLeftGripper.setPosition(UPPER_LEFT_GLYPH_ARM_INITIAL_POSITION);
            upperRightGripper.setPosition(UPPER_RIGHT_GLYPH_ARM_INITIAL_POSITION);
            lowerLeftGripper.setPosition(LOWER_LEFT_GLYPH_ARM_INITIAL_POSITION);
            lowerRightGripper.setPosition(LOWER_RIGHT_GLYPH_ARM_INITIAL_POSITION);
        }
        else {
            openGlyphGripperWider();
        }

        try {
            led = hardwareMap.dcMotor.get("led");
            led.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        } catch(Exception e) {
            logInfo(this.telemetry, "Led power control failed.", e.getMessage());
        }

        glyphLift = hardwareMap.dcMotor.get("glyphLift");
        glyphLift.setDirection(DcMotorSimple.Direction.REVERSE);
        glyphLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        glyphLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        relicSlider = hardwareMap.dcMotor.get("relicSlider");
        relicSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        relicSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        try {
            relicClaw = hardwareMap.servo.get("relicClaw");
            relicClaw.setPosition(RELIC_CLAW_INITIAL_POSITION);
        } catch(Exception e) {
            logInfo(this.telemetry, "Init relic init failed", e.getMessage());
        }

        try {
            relicElbow = hardwareMap.servo.get("relicElbow");
            relicElbow.setPosition(RELIC_ELBOW_INITIAL_POSITION);
        } catch(Exception e) {
            logInfo(this.telemetry, "Init relic elbow servo failed", e.getMessage());
        }

        logInfo(null, "Init Servos", " Servos are initialized ...");

        telemetry.addData("Robot initialized", "Ready to go...");
    }

    public void initServosForTeleOps() {
        try {
            if(relicClawholder != null) relicClawholder.setPosition(RELIC_CLAWHOLDER_INITIAL_POSITION);
            if(glyphHolder != null) glyphHolder.setPosition(GLYPH_TOP_HOLDER_INITIAL_POSITION);
            if(relicClaw != null) relicClaw.setPosition(RELIC_CLAW_INITIAL_POSITION);
            if(relicElbow != null) relicElbow.setPosition(RELIC_ELBOW_INITIAL_POSITION);
            if(longArm != null) longArm.setPosition(JEWEL_PUSHER_LONG_ARM_TELEOPS_POSITION);

            openGlyphGripperMidWide();
        }
        catch(Exception e) {
            logInfo(this.telemetry, "Init servos failed", e.getMessage());
        }
    }

    private void initMotors(HardwareMap hardwareMap) {
        lf = hardwareMap.dcMotor.get("lf");
        lr = hardwareMap.dcMotor.get("lr");
        rf = hardwareMap.dcMotor.get("rf");
        rr = hardwareMap.dcMotor.get("rr");

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lr.setDirection(DcMotorSimple.Direction.REVERSE);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, lf, lr, rr, rf);

        logInfo(null, "Init Drive Motors", " Drive Motors are initialized ...");
    }

    private void initGyro(HardwareMap hardwareMap) {
        ElapsedTime timer = new ElapsedTime();

        Log.i(this.getClass().getSimpleName(), timer.time(TimeUnit.MILLISECONDS) + " | init ...");

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        logInfo(null, "Init Imu", timer.time(TimeUnit.MILLISECONDS) + " | imu initialized ...");
    }

    private void initRangeSensor(HardwareMap hardwareMap, AllianceColor alliance) {
        try {
            //if(alliance == AllianceColor.RED) {
            //    x1RangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "x1Range");
            //} else {
                x1RangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "x1Range");
                x2RangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "x2Range");
            //}

            logInfo(null, "Init Range Sesnor", " Range Sensor x1 and x2 are initialized ...");
        }
        catch(Exception e) {
            Log.e(this.getClass().getSimpleName(), "Range Sesnor failed: " + e.getMessage());
        }
    }

    public void onStart() {
        setMode(DcMotor.RunMode.RUN_USING_ENCODER, lf, lr, rr, rf, glyphLift);
    }

    public void resetDriveMotors() {
        stopDriveMotors();
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER, lf, lr, rr, rf);
    }

    public void stopGlyphLiftMotor() {
        glyphLift.setPower(0.0);
    }

    /**
     * This should be used only on the {@link OpMode} stop
     * Don't use this to stop motors
     */
    public void onStop() {
        stopDriveMotors();
        stopLiftMotors();
    }

    private interface Stoppable {
        public boolean stopped();
    }

    private int averageRemainingTicks(DcMotor... ms) {
        int total = 0;
        int count = 0;
        for (DcMotor m : ms) {
            if (m.getMode() == DcMotor.RunMode.RUN_TO_POSITION && 100 < Math.abs(m.getTargetPosition())) {
                total += Math.abs(m.getTargetPosition() - m.getCurrentPosition());
                count += 1;
            }
        }
        return 0 == count ? 0 : total / count;
    }

    private static int SLOW_DOWN_HERE = 1120;
    private static double ARBITRARY_SLOW_SPEED = .3;
    private boolean slowedDown = false;
    private void encoderDriveSlowdown() {
        if (! slowedDown) {
            if (lf.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
                int remaining = averageRemainingTicks(lf, lr, rf, rr);
                if (remaining < SLOW_DOWN_HERE) {
                    slowedDown = true;
                    setPower(ARBITRARY_SLOW_SPEED, lf, lr, rf, rr);
                }
            }
        }
    }

    /**
     * Use this to avoid the mecanum wheels slippage
     * start slow and end slow
     */
    public void loop() {
        //setMode(DcMotor.RunMode.RUN_USING_ENCODER, lf, lr, rr, rf, glyphLift);
        encoderDriveSlowdown();
        manageEncoderAccelleration(lf, lr, rf, rr);
    }

    /**
     * We will add more sendor data here
     */
    public void updateSensorTelemetry() {

        double currentAvgRemaning = averageRemainingTicks(lf, lr, rf, rr);
        double currentMovingAvg = driveEncodersMovingAvg.next(currentAvgRemaning);

        logInfo(telemetry,"Encoder Remain", String.format("\t%.1f\t%.1f\t%.1f", currentAvgRemaning, currentMovingAvg, previousDriveAvgEncoder));
        previousDriveAvgEncoder = currentMovingAvg;

        logInfo(telemetry,"EncodersC", String.format("\t%d\t%d\t%d\t%d",
                lf.getCurrentPosition(),
                lr.getCurrentPosition(),
                rf.getCurrentPosition(),
                rr.getCurrentPosition()));

        logInfo(telemetry,"EncodersT", String.format("\t%d\t%d\t%d\t%d",
                lf.getTargetPosition(),
                lr.getTargetPosition(),
                rf.getTargetPosition(),
                rr.getTargetPosition()));
    }

    /**
     *  get the maximum absolute value of some number of arguments
     */
    private static double getMax(double... xs) {
        double ret = 0.0;
        for (double x : xs) {
            ret = Math.max(ret, Math.abs(x));
        }
        return ret;
    }

    public void moveGlyphLift(double power) {

//        if(glyphLift.getCurrentPosition() < -50 && power < 0.0) {
//            return;
//        }
//
//        if(glyphLift.getCurrentPosition() > 5000 && power > 0.0) {
//            return;
//        }

        this.glyphLift.setPower(Range.clip(power, -0.70, 0.90));

//        if(glyphLift.getCurrentPosition() < 300 && power < 0.0) {
//            this.glyphLift.setPower(-0.25);
//        } else {
//            this.glyphLift.setPower(Range.clip(power, -0.90, 0.90));
//        }
    }

    public void forceMoveGlyphLiftDown(double power) {
        this.glyphLift.setPower(Range.clip(power, -0.30, 0.30));
    }

    public void resetGlyphLiftEncoder(double power) {
        this.glyphLift.setPower(0.0);
        this.glyphLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.glyphLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void openGlyphGripper() {
        openUpperGlyphGripper();
        openLowerGlyphGripper();
    }

    public void closeGlyphGripper() {
        closeUpperGlyphGripper();
        closeLowerGlyphGripper();
    }

    public void openGlyphGripperWider() {
        openUpperGlyphGripperMidWide();
        openLowerGlyphGripperWide();
        this.glyphHolder.setPosition(GLYPH_TOP_HOLDER_OPEN_POSITION);
    }

    public void openGlyphGripperMidWide() {
        openUpperGlyphGripperMidWide();
        openLowerGlyphGripperWide();
        this.glyphHolder.setPosition(GLYPH_TOP_HOLDER_OPEN_POSITION);
    }

    // open/close upper gripper
    //---------------------------------------------------------------------
    public void openUpperGlyphGripper() {
        upperLeftGripper.setPosition(UPPER_LEFT_GLYPH_ARM_OPEN_POSITION);
        upperRightGripper.setPosition(UPPER_RIGHT_GLYPH_ARM_OPEN_POSITION);
        glyphHolder.setPosition(GLYPH_TOP_HOLDER_OPEN_POSITION);
    }

    public void closeUpperGlyphGripper() {
        upperLeftGripper.setPosition(UPPER_LEFT_GLYPH_ARM_CLOSE_POSITION);
        upperRightGripper.setPosition(UPPER_RIGHT_GLYPH_ARM_CLOSE_POSITION);
        this.glyphHolder.setPosition(GLYPH_TOP_HOLDER_CLOSE_POSITION);
    }

    public void alignStackedGlyphs() {
        // try to close and open gripper multiple times
        // press the top glyphs into squared position
        int idx = 3;
        while(idx-- >0) {
            glyphHolder.setPosition(GLYPH_TOP_HOLDER_OPEN_POSITION);
            upperLeftGripper.setPosition(UPPER_LEFT_GLYPH_ARM_CLOSE_POSITION);
            upperRightGripper.setPosition(UPPER_RIGHT_GLYPH_ARM_CLOSE_POSITION);

            sleepInLiearMode(150);

            upperLeftGripper.setPosition(UPPER_LEFT_GLYPH_ARM_OPEN_POSITION);
            upperRightGripper.setPosition(UPPER_RIGHT_GLYPH_ARM_OPEN_POSITION);
            sleepInLiearMode(150);
        }

        closeUpperGlyphGripper();
    }


    public void openUpperGlyphGripperWide() {
        upperLeftGripper.setPosition(UPPER_LEFT_GLYPH_ARM_WIDE_OPEN_POSITION);
        upperRightGripper.setPosition(UPPER_RIGHT_GLYPH_ARM_WIDE_OPEN_POSITION);
        glyphHolder.setPosition(GLYPH_TOP_HOLDER_OPEN_POSITION);
    }

    public void openUpperGlyphGripperMidWide() {
        upperLeftGripper.setPosition(UPPER_LEFT_GLYPH_ARM_MEDIUM_OPEN_POSITION);
        upperRightGripper.setPosition(UPPER_RIGHT_GLYPH_ARM_MEDIUM_OPEN_POSITION);
        glyphHolder.setPosition(GLYPH_TOP_HOLDER_OPEN_POSITION);
    }

    // open/close lower gripper
    //-------------------------------------------------------------------
    public void openLowerGlyphGripper() {
        lowerLeftGripper.setPosition(LOWER_LEFT_GLYPH_ARM_OPEN_POSITION);
        lowerRightGripper.setPosition(LOWER_RIGHT_GLYPH_ARM_OPEN_POSITION);
    }

    public void closeLowerGlyphGripper() {
        lowerLeftGripper.setPosition(LOWER_LEFT_GLYPH_ARM_CLOSE_POSITION);
        lowerRightGripper.setPosition(LOWER_RIGHT_GLYPH_ARM_CLOSE_POSITION);

        openGlyphLiftStopper();
    }

    public void openLowerGlyphGripperWide() {
        lowerLeftGripper.setPosition(LOWER_LEFT_GLYPH_ARM_WIDE_OPEN_POSITION);
        lowerRightGripper.setPosition(LOWER_RIGHT_GLYPH_ARM_WIDE_OPEN_POSITION);
    }

    public void openLowerGlyphGripperMidWide() {
        lowerLeftGripper.setPosition(LOWER_LEFT_GLYPH_ARM_MEDIUM_OPEN_POSITION);
        lowerRightGripper.setPosition(LOWER_RIGHT_GLYPH_ARM_MEDIUM_OPEN_POSITION);
    }

    public void pickupGlyphInAuto() {
        // close the grabber
        closeUpperGlyphGripper();

        // sleep for 1 second
        sleepInLiearMode(1000);

        // move up glyph
        this.glyphLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleepInLiearMode(200);

        this.glyphLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.glyphLift.setTargetPosition(GLYPH_LIFT_PICKUP_GLYPH_IN_AUTO_POSITION);
        this.glyphLift.setPower(0.5);

        ElapsedTime timer = new ElapsedTime();
        // wait for 2 seconds, if not reached, move on anyway
        while(busy(glyphLift) && timer.time(TimeUnit.SECONDS) < 2 &&
                this.linearOpMode != null && this.linearOpMode.opModeIsActive()) {
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        //this.glyphLift.setPower(0.0);
        //this.glyphLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setGlyphLiftToPosition(int position) {
        // move up glyph
        this.glyphLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.glyphLift.setTargetPosition(position);
        this.glyphLift.setPower(
                ((position - this.glyphLift.getCurrentPosition())> 0?1.0:-1.0)*0.8);
    }

    public void holdGlyphLiftPosition(int position) {
        // move up glyph
        this.glyphLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.glyphLift.setTargetPosition(position);
        this.glyphLift.setPower(0.2);
    }

    public boolean isGlyphLiftTargetReached() {
        if(!(glyphLift.isBusy()) ||
                Math.abs(this.glyphLift.getCurrentPosition() - this.glyphLift.getTargetPosition())< 50) {
            this.glyphLift.setPower(0.0);
            this.glyphLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            return true;
        }
        return false;
    }

    public void setGlyphLiftToRunEncoderMode() {
        this.glyphLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Move the glyph lift to zero position and get ready for the next task
     */
    public void resetGlyphLift() {

        this.glyphLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.glyphLift.setTargetPosition(-100);
        this.glyphLift.setPower(0.5);

        ElapsedTime timer = new ElapsedTime();
        while( glyphLift.isBusy() && timer.time(TimeUnit.SECONDS) < 3
                && this.linearOpMode != null && this.linearOpMode.opModeIsActive()) {
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        this.glyphLift.setPower(0.0);
        this.glyphLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Autonomous is using LinearOpMode, it's ok to call Thrad.sleep()
     *
     * @param milliSeconds
     */
    private void sleepInLiearMode(long milliSeconds) {
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        while(this.linearOpMode != null && this.linearOpMode.opModeIsActive()
                && timer.time() < milliSeconds) {
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    /**
     * Represent the 4 mecanum wheels speed
     *
     * 1. lf
     * 2. rf
     * 3. lr
     * 4. rr
     */
    private static class WheelsSpeed {
        public double lf, lr, rf, rr;

        public WheelsSpeed(double lf, double rf, double lr, double rr) {
            this.lf = lf;
            this.rf = rf;
            this.lr = lr;
            this.rr = rr;
        }
    }

    /**
     * Calculate the mecanum wheep speed for v1,v2,v3 and v4
     *
     *  Trigonometry version:
     *
     * V1 = Vd * sin(td + PI/4) + Vt
     * V2 = Vd * cos(td + PI/4) - Vt
     * V3 = Vd * cos(td + PI/4) + Vt
     * V4 = Vd * sin(td + PI/4) - Vt
     *
     * @param direction -- theta
     * @param velocity
     * @param rotation
     * @return
     */
    private WheelsSpeed getWheelsSpeed(double direction, double velocity, double rotation) {
        final double vd = velocity;
        final double td = direction;
        final double vt = rotation;

        double s =  Math.sin(td + Math.PI / 4.0);
        double c = Math.cos(td + Math.PI / 4.0);
        double m = Math.max(Math.abs(s), Math.abs(c));
        s /= m;
        c /= m;

        final double v1 = vd * s + vt;
        final double v2 = vd * c - vt;
        final double v3 = vd * c + vt;
        final double v4 = vd * s - vt;

        // Ensure that none of the values go over 1.0. If none of the provided values are
        // over 1.0, just scale by 1.0 and keep all values.
        double scale = getMax(1.0, v1, v2, v3, v4);

        return new WheelsSpeed(v1 / scale, v2 / scale, v3 / scale, v4 / scale);
    }

    /**
     * Main interface to take translation and rotation from OpModes
     *
     * @param direction
     * @param velocity
     * @param rotationVelocity
     */
    public void drive(double direction, double velocity, double rotationVelocity) {
        WheelsSpeed w = getWheelsSpeed(direction, velocity, rotationVelocity);
        lf.setPower(w.lf);
        rf.setPower(w.rf);
        lr.setPower(w.lr);
        rr.setPower(w.rr);

        if(Math.abs(direction) >0 || Math.abs(velocity) >0 || Math.abs(rotationVelocity) > 0) {
            //logInfo(telemetry, "WheelsSpeed", String.format(Locale.US, "%.2f %.2f %.2f", direction, velocity, rotationVelocity));
//            logInfo(telemetry, "Powers", String.format(Locale.US, "%.2f %.2f %.2f %.2f", w.lf, w.rf, w.lr, w.rr));
//            logInfo(telemetry, "Encoders", String.format(Locale.US, "%d %d %d %d",
//                    lf.getCurrentPosition(),
//                    rf.getCurrentPosition(),
//                    lr.getCurrentPosition(),
//                    rr.getCurrentPosition()));
//
//            logInfo(telemetry, "Angles", String.format(Locale.US, "%.2f %.2f %.2f ",
//                    getRawsAngle(IMUAngleOrientation.HEADING),
//                    getRawsAngle(IMUAngleOrientation.ROLL),
//                    getRawsAngle(IMUAngleOrientation.PITCH)
//            ));
        }

//        if(Math.abs(glyphLift.getPower()) > 0 ) {
//            logInfo(telemetry, "Lift: " + glyphLift.getMode(), String.format(Locale.US, "%d %.2f",
//                    glyphLift.getCurrentPosition(), glyphLift.getPower()));
//        }
    }

    double getRawHeadingAngle() {
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        return AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
    }

    double getRawsAngle(IMUAngleOrientation type) {
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        switch(type) {
            case HEADING:
                return AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
            case ROLL:
                return AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.secondAngle));
            default:
                return AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.thirdAngle));
        }
    }

    /**
     * return 180 to -180 (turn left is positive & turn right is negative)
     *
     * @return
     */
    public double getHeadingAngle(){
        if(imu == null)  return -0.0;

        double tempAng = getRawHeadingAngle();
        if(tempAng > 180.0) {
            tempAng = tempAng - 360;
        }
        else if(tempAng < -180.0) {
            tempAng = tempAng + 360;
        }
        return tempAng;
    }

    enum IMUAngleOrientation {
        HEADING,
        ROLL,
        PITCH
    }

    /// Shut down all motors
    public void stopDriveMotors() {
        lf.setPower(0.0);
        lr.setPower(0.0);

        rf.setPower(0.0);
        rr.setPower(0.0);
    }

    public void stopLiftMotors() {
        glyphLift.setPower(0.0);
    }

    public void moveGlyphLift(GlyphLiftLevel level) {

    }

    public enum GlyphLiftLevel {
        LEVEL0,
        LEVEL1,
        LEVEL2,
        LEVEL3
    }

    void setEncoderDrivePower(double p) {
        encoder_drive_power = p;
    }

    void clearEncoderDrivePower() {
        encoder_drive_power = ENCODER_DRIVE_POWER;
    }

    private void setMode(DcMotor.RunMode mode, DcMotor... ms) {
        for (DcMotor m : ms) {
            m.setMode(mode);
        }
    }

    private void setPower(double p, DcMotor... ms) {
        for (DcMotor m : ms) {
            m.setPower(p);
        }
    }

    private void setTargetPosition(int pos, DcMotor... ms) {
        for (DcMotor m : ms) {
            m.setTargetPosition(pos);
        }
    }

    // 4 encoders, with 6 counts each
    public static final int ENCODERS_CLOSE_ENOUGH = 20;

    /**
     * Check to see if robot should stop when using SET_TO_POSITION mode
     *
     * @param motors
     * @return
     */
    private boolean busy(DcMotor... motors) {
        int total = 0;
        for (DcMotor m : motors) {
            if (m.isBusy()) {
                final int c = Math.abs(m.getCurrentPosition());
                final int t = Math.abs(m.getTargetPosition());
                total += Math.max(0, t - c);
            }
        }
        return total > ENCODERS_CLOSE_ENOUGH;
    }

    public boolean driveMotorsBusy() {
        return busy(lf, lr, rf, rr);
    }

    /**
     * Use SET_TO_POSITION to drive the robot to a specific distance
     * @param direction
     * @param tiles
     */
    public void encoderDriveTiles(double direction, double tiles) {
        encoderDriveInches(direction, 24.0 * tiles);
    }

    public void encoderDriveInches(double direction, double inches) {
        final WheelsSpeed w = getWheelsSpeed(direction, 1.0, 0.0);
        final int ticks = (int)(inches * TICKS_PER_INCH);
        encoderDrive(ticks * w.lf, ticks * w.rf, ticks * w.lr, ticks * w.rr);
    }

    public void encoderDriveCM(double direction, double cm) {
        direction %= Math.PI * 2.0;
        final WheelsSpeed w = getWheelsSpeed(direction, 1.0, 0.0);
        final int ticks = (int)(cm * TICKS_PER_CM);
        encoderDrive(ticks * w.lf, ticks * w.rf, ticks * w.lr, ticks * w.rr);
    }

    private void encoderDrive(double lft, double rft, double lrt, double rrt) {
        encoderDrive((int) lft, (int) rft, (int) lrt, (int) rrt);
    }

    private void encoderDrive(int lft, int rft, int lrt, int rrt) {
        // stop the motor and reset the encoders
        setPower(0.0, lf, lr, rf, rr);
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, lf, lr, rf, rr);

        // set the target position to all 4 motors
        setTargetPosition(lft, lf);
        setTargetPosition(rft, rf);
        setTargetPosition(lrt, lr);
        setTargetPosition(rrt, rr);
        setMode(DcMotor.RunMode.RUN_TO_POSITION, lf, rf, lr, rr);

        setPower(encoder_drive_power, lf, lr, rf, rr);
        slowedDown = false;
    }

    // All motors start out at ENCODER_DRIVE_POWER power. Once we get one revolution
    // in, go ahead and speed up. When we get within a revolution of the end of our
    // run, start slowing down. The idea here is to avoid slip.
    private static final int ACCEL_THRESHOLD = 1120 * 24 / 32; // one wheel revolution, for starters
    private boolean atSteadyState = false;
    private void manageEncoderAccelleration(DcMotor... ms) {
        if (encoder_drive_power > ENCODER_DRIVE_POWER) {
            int current = 0, remaining = 0, count = 0;

            ArrayList<DcMotor> driving = new ArrayList<>();
            for (DcMotor m : ms) {
                if (m.getMode() == DcMotor.RunMode.RUN_TO_POSITION && 0 != m.getTargetPosition()) {
                    driving.add(m);
                    current += m.getCurrentPosition();
                    remaining += Math.abs(m.getCurrentPosition() - m.getTargetPosition());
                    count++;
                }
            }

            if (0 < driving.size()) {
                current /= count;
                remaining /= count;

                double power = encoder_drive_power;
                double dp = encoder_drive_power - ENCODER_DRIVE_POWER;

                if (remaining < ACCEL_THRESHOLD) {
                    atSteadyState = false;
                    power = ENCODER_DRIVE_POWER + dp * ((double)remaining / (double)ACCEL_THRESHOLD);
                    for (DcMotor m : driving) {
                        m.setPower(power);
                    }
                } else if (current < ACCEL_THRESHOLD) {
                    atSteadyState = false;
                    power = ENCODER_DRIVE_POWER + dp * ((double)current / (double)ACCEL_THRESHOLD);
                    for (DcMotor m : driving) {
                        m.setPower(power);
                    }
                }  else {
                    if (! atSteadyState) {
                        for (DcMotor m : driving) {
                            m.setPower(power);
                        }
                        atSteadyState = true;
                    }
                }

            }
        }
    }

    public void resetDriveMotorModes() {
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, lf, lr, rf, rr);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER, lf, lr, rf, rr);
    }

    public void disableEncoders() {
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER, lf, lr, rf, rr);
    }

    /**
     * log the message to both telemetry and file
     *
     * @param telemetry
     * @param tag
     * @param message
     */
    public void logInfo(Telemetry telemetry, String tag, String message) {
        Log.i("TechNova: " + this.getClass().getSimpleName(), tag + " | " + message);

        if(telemetry != null) {
            telemetry.addData(tag, message);
        }
    }

    public void logInfo(Telemetry telemetry, String tag, String message, boolean inLogCat) {
        if(inLogCat) {
            Log.i("TechNova: " + this.getClass().getSimpleName(), tag + " | " + message);
        }

        if(telemetry != null) {
            telemetry.addData(tag, message);
        }
    }

    /**
     * @return the raw heading along the desired axis
     */
    private double getRawHeading() {
        return angles.firstAngle;
    }

    /**
     * @return the robot's current heading in radians
     */
    public double getRawHeadingRadian() {
        return (getRawHeading() - headingOffset) % (2.0 * Math.PI);
    }

    /**
     * @return the robot's current heading in degrees
     */
    public double getRawHeadingDegrees() { return Math.toDegrees(getRawHeadingRadian()); }

    /**
     * Set the current heading to zero.
     */
    public void resetHeading() {
        headingOffset = getRawHeading();
    }

    public double getX1Distance() {
        if(x1RangeSensor != null) {
            try {
                double distance = x1RangeSensor.getDistance(DistanceUnit.CM);
                if (distance < 225.0 && distance > 25.0) {
                    prevX1Distance = distance;
                }
            }catch(Exception e) {
                Log.e(this.getClass().getSimpleName(), "X1 Range Failed: " + e.getMessage());
                prevX1Distance = 0.0;
            }
        } else {
            return 0.0;
        }

        return prevX1Distance;
    }

    public double getX2Distance() {
        if(x2RangeSensor != null) {
            try {
                double distance = x2RangeSensor.getDistance(DistanceUnit.CM);
                if (distance < 225.0 && distance > 25.0) {
                    prevX2Distance = distance;
                }
            }catch(Exception e) {
                Log.e(this.getClass().getSimpleName(), "X2 Range Failed: " + e.getMessage());
                prevX2Distance = 0.0;
            }
        } else {
            return 0.0;
        }

        return prevX2Distance;
    }

    public double getYDistance() {
        if(yRangeSensor != null) {
            try {
                double distance = yRangeSensor.getDistance(DistanceUnit.CM);
                if (distance < 225.0 && distance > 25.0) {
                    prevX1Distance = distance;
                }
            }catch(Exception e) {
                Log.e(this.getClass().getSimpleName(), "Y Range Failed: " + e.getMessage());
                prevX1Distance = 0.0;
            }
        }
        else {
            return 0.0;
        }

        return prevYDistance;
    }

    public int getGlyphLiftPosition() {
        return this.glyphLift.getCurrentPosition();
    }

    public void turnOnBlueLed() {
        if(led != null) {
            led.setPower(0.95);
            isBlueLedOn = true;
        }
    }

    public void turnOffBlueLed() {
        if(led != null) {
            led.setPower(0.0);
            isBlueLedOn = false;
        }
    }

    public void turnOnGreenLed() {
        if(led != null) {
            led.setPower(-0.95);
            isGreenLedOn = true;
        }
    }

    public void turnOffGreenLed() {
        if(led != null) {
            led.setPower(0.0);
            isGreenLedOn = false;
        }
    }

    public boolean isBlueLedOn() {
        return this.isBlueLedOn;
    }

    public boolean isGreenLedOn() {
        return this.isGreenLedOn;
    }

    public double getRangeSensorVol() {
        if(rangeSensor != null) {
            return rangeSensor.getVoltage();
        }

        return 0.0;
    }

    public void moveRelicSlider(double power) {
        relicSlider.setPower(Range.clip(power, -0.30, 1.0));
    }

    public void grabRelic() {
        if(relicClaw != null) {
            relicClaw.setPosition(RELIC_CLAW_CLOSE_POSITION);
        }
    }

    public void releaseRelic() {
        if(relicClaw != null) {
            relicClaw.setPosition(RELIC_CLAW_OPEN_POSITION);
        }
    }

    public void raiseRelicOverTheWall () {
        if(relicElbow != null) {
            relicElbow.setPosition(RELIC_ELBOW_UP_POSITION);
        }
    }

    public void prepareRelicLanding () {
        if(relicElbow != null) {
            relicElbow.setPosition(RELIC_ELBOW_RELEASE_POSITION + 0.15);
        }
    }

    public void setRelicElbowPosition(double position) {
        if(relicElbow != null) {
            relicElbow.setPosition(position);
        }
    }

    public double getRelicElbowPosition() {
        if(relicElbow != null) {
            return relicElbow.getPosition();
        }

        return 0.0;
    }

    public double getGlyphLiftStopperPosition() {
        if(glyphLiftStopper != null) {
            return glyphLiftStopper.getPosition();
        }

        return 0.0;
    }

    public void releaseClaw() {
        if (relicClawholder != null) {
            relicClawholder.setPosition(RELIC_CLAWHOLDER_RELEASE_POSITION);
        }

//        if(relicClaw != null) {
//            relicClaw.setPosition(RELIC_CLAW_OPEN_POSITION);
//        }
    }

    public void closeRelicClawHolder() {
        if (relicClawholder != null) {
            relicClawholder.setPosition(RELIC_CLAWHOLDER_INITIAL_POSITION);
        }
    }

    public void openGlyphLiftStopper() {
        if(glyphLiftStopper != null) {
            glyphLiftStopper.setPosition(GLYPH_LIFT_STOPPER_OPEN_POSITION);
        }
    }

    public void closeGlyphLiftStopper() {
        if(glyphLiftStopper != null) {
            glyphLiftStopper.setPosition(GLYPH_LIFT_STOPPER_CLOSE_POSITION);
        }
    }

    public void setGlyphLiftStopperPosition(double position) {
        if(glyphLiftStopper != null) {
            glyphLiftStopper.setPosition(position);
        }
    }

    public void logMotorEncoders(Telemetry telemetry, boolean inLogCat) {
        logInfo(telemetry, "Mode Encoder: ", lf.getCurrentPosition()
         + " | " + rf.getCurrentPosition()
         + " | " + lr.getCurrentPosition()
         + " | " + rr.getCurrentPosition(), inLogCat);
    }

    public double getBatteryVoltage() {
        if(this.voltageSensor != null) {
            return this.voltageSensor.getVoltage();
        }

        return 0.0;
    }

    public boolean isGlyphTouched() {
        if(this.glyphDistance != null) {
            return this.glyphDistance.getDistance(DistanceUnit.INCH) < 2.5;
        }

        return false;
    }

    public double getGlyphDistance() {
            return this.glyphDistance.getDistance(DistanceUnit.INCH);
    }

}

