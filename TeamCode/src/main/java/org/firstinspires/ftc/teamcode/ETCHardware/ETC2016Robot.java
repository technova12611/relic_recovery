package org.firstinspires.ftc.teamcode.ETCHardware;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbDcMotorController;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static org.firstinspires.ftc.teamcode.ETCHardware.ETCConstants.SPEED_GEAR_RATIO_FACTOR;

/**
 * RegularWheelRobot for 2016-2017 Velocity Vortex FTC Game
 */
public class ETC2016Robot {

    public static enum BeaconColor {
        RED,
        BLUE,
        UNKNOWN,
        BAD_SENSOR
    }

    public DcMotor leftDriveMotor;
    public DcMotor rightDriveMotor;
    public DcMotor collectorMotor;
    public DcMotor lifterMotor;

    // for ball shooter
    public ETCBallShooter ballShooter;

    private DcMotor shooterMotor;
    private DcMotor tensionMotor;
    public TouchSensor shooterSwitch;
    private TouchSensor tensionSwitch;
    public Servo stopperServo;

    private Servo pusherServo;
    public Servo releaseServo;

    public ETCBeaconButtonPusher buttonPusher;
    public DigitalChannel pusherLimitSensor;
    public ETCAdafruitIMU gyroSensor;

    public ModernRoboticsI2cColorSensor colorSensorBottomFront;
    public ModernRoboticsI2cColorSensor colorSensorBottomBack;
    public ModernRoboticsI2cColorSensor colorSensorBeacon;
    public ModernRoboticsI2cColorSensor colorSensorParticleRight;
    public ModernRoboticsI2cColorSensor colorSensorParticleLeft;

    private FtcI2cDeviceState colorFrontState;
    private FtcI2cDeviceState colorBackState;
    private FtcI2cDeviceState colorBeaconState;
    private FtcI2cDeviceState colorParticleRightState;
    private FtcI2cDeviceState colorParticleLeftState;

    private FtcI2cDeviceState gyroSensorState;

    private OpticalDistanceSensor lineDetectorODS;

    private BNO055IMU imu;
    Orientation angles;
    private double headingOffset = 0.0d;

    protected I2cDeviceSynchImpl rangeReaderSync;
    protected ModernRoboticsI2cRangeSensor rangeSensor;

    public int lastLeftMotorEncoderValue = 0;
    public int lastRightMotorEncoderValue = 0;

    public double stateBeginHeadingAngle = 0.0;
    public double stateBeginDistance = 0.0;

    public double previousRangeDistance = 0.0;

    public DeviceInterfaceModule dim;
    public DeviceInterfaceModule dim2;

    protected static final double HEADING_ANGLE_TOLERANCE = 1.0;

    public static final int LED_CHANNEL = 5;

    public static final int    DIM_BLUE_LED    = 0;     // Blue LED channel on DIM
    public static final int    DIM_RED_LED     = 1;     // Red LED Channel on DIM

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 3/4 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.7;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1.0 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.025;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.10;     // Larger is more responsive, but also less stable

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    Telemetry telemetry         = null;
    private ElapsedTime period  = new ElapsedTime();

    public boolean hasGyroSensor = true;

    protected ModernRoboticsUsbDcMotorController motorController;

    /* Constructor */
    public ETC2016Robot(){
    }

    /* Initialize standard Hardware interfaces */
    public boolean init(HardwareMap ahwMap, Telemetry atelemetry, boolean isAutonomous) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        telemetry = atelemetry;
        boolean success = true;

        boolean dim2Success = false;

        if(isAutonomous && hasGyroSensor) {
            success &= initAdafruitIMU();
        }

        dim2Success = success;

        dim = hwMap.deviceInterfaceModule.get("dim");
        try {
            dim2 = hwMap.deviceInterfaceModule.get("dim2");
        }
        catch(Exception e) {
            telemetry.addData("0000", "Couldn't get dim2");
            dim2 = null;
        }

        try {
            // one of the motor needs to run reverse due to the direction it's mounted
            leftDriveMotor = hwMap.dcMotor.get("chassisLeft");
            leftDriveMotor.setDirection(DcMotor.Direction.REVERSE);
            leftDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        catch(Exception e) {
            telemetry.addData("000", "Couldn't get Left Drive");
            success = false;
        }

        try {
            rightDriveMotor = hwMap.dcMotor.get("chassisRight");
            rightDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        catch(Exception e) {
            telemetry.addData("001", "Couldn't get right Drive");
            success = false;
        }

        try {
            collectorMotor = hwMap.dcMotor.get("collectorMotor");
        }
        catch(Exception e) {
            telemetry.addData("002", "Couldn't get Collector Motor");
            success = false;
        }

        try {
            shooterMotor = hwMap.dcMotor.get("shooterMotor");
            shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        catch(Exception e) {
            telemetry.addData("003", "Couldn't get Shooter Motor");
            success = false;
        }

        try {
            tensionMotor = hwMap.dcMotor.get("tensionMotor");
            tensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        catch(Exception e) {
            telemetry.addData("004", "Couldn't get Tension Motor");
            success = false;
        }

        try {
            lifterMotor = hwMap.dcMotor.get("lifterMotor");
            lifterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        catch(Exception e) {
            telemetry.addData("002", "Couldn't get Lifter Motor");
            success = false;
        }

        try {
            pusherServo = hwMap.servo.get("pusherServo");
        }catch(Exception e) {
            telemetry.addData("005", "Couldn't get Pusher Servo");
            success = false;
        }

        try {
            stopperServo = hwMap.servo.get("stopperServo");
            //stopperServo.setPosition(ETCConstants.STOPPER_CLOSE_POSITION);
        }
        catch(Exception e) {
            telemetry.addData("015", "Couldn't get Stopper Servo");
            success = false;
        }

        try {
            releaseServo = hwMap.servo.get("releaseServo");
            releaseServo.setPosition(1.0);
        }
        catch(Exception e) {
            telemetry.addData("018", "Couldn't get Release Servo");
            success = false;
        }

        success &=initShooterTouchSensor();
        success &=initTensionTouchSensor();

        if(isAutonomous) {
            success &= initBottomFrontColorSensor();
            success &= initBottomBackColorSensor();
            success &= initRangeSensor();
            success &= initBeaconColorSensor();
        }

        success &= initColorSensorParticleRight();
        success &= initColorSensorParticleLeft();

        ballShooter = new ETCBallShooter(shooterMotor,tensionMotor, shooterSwitch,
                tensionSwitch, stopperServo, telemetry);
        ballShooter.init();

        pusherLimitSensor =  hwMap.get(DigitalChannel.class, "pusherSwitch");
        buttonPusher = new ETCBeaconButtonPusher(pusherServo, pusherLimitSensor);

        if(isAutonomous) {
            buttonPusher.init();
        } else {
            buttonPusher.teleop_init();
        }

        if(dim != null) {
            dim.setLED(DIM_BLUE_LED, success); // Blue for success
            dim.setLED(DIM_RED_LED, !success); // Red for failed
        }

        if(dim2 != null) {
            dim2.setLED(DIM_BLUE_LED, dim2Success); // Blue for success
            dim2.setLED(DIM_RED_LED, !dim2Success); // Red for failed
        }

        motorController = (ModernRoboticsUsbDcMotorController)hwMap.dcMotorController.get("Shooter");

        return success;
    }

    private boolean initAdafruitIMU() {
        if(imu != null) {
            return true;
        }
        boolean success = true;
        long systemTime = System.currentTimeMillis();
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "gyroSensor");
        if(imu != null) {
            imu.initialize(parameters);

            headingOffset = getRawHeadingAngle();
            telemetry.addData("00", "heading offset:" + headingOffset);

            long elapsedTime = System.currentTimeMillis() - systemTime;
            //String initStatus = (headingOffset == 0.0 ? " | Failed, Stop and Restart" : " | Success");
            logInfo("Gyro sensor initialized in: " +
                    elapsedTime + " ms." + " | Heading: " + headingOffset);

            telemetry.addData("01. IMU init: ",
                    elapsedTime + "  | Heading: " + headingOffset);
        }
        else {
            telemetry.addData("01.", "Gyro sensor not configured");
        }
        return success;
    }

    private boolean initBottomFrontColorSensor() {
        boolean success = true;
        try
        {
            colorSensorBottomFront = (ModernRoboticsI2cColorSensor) hwMap.colorSensor.get("colorSensorBottomFront");
            colorSensorBottomFront.enableLed(true);

            if(colorSensorBottomFront.red() == 255) {
                colorSensorBottomFront = null;
                String error = " ** Bottom Color Sensor Front Failed. " + " Stop and Restart !!!";
                telemetry.addData("02. Color: ", error);

                Log.e(this.getClass().getName(), error);
            }

            //colorFrontState = new FtcI2cDeviceState(colorSensorBottomFront);

            telemetry.addData ("03.", "Color Bottom Front Initialized");
            logInfo("color sensor bottom front initialized");
        }
        catch (Exception e)
        {
            telemetry.addData("03.", "color sensor bottom front failed");
            Log.e(this.getClass().getSimpleName(), "Exception: " + e.getMessage());
            colorSensorBottomFront = null;
            success = false;
        }

        return success;
    }

    public void setColorSensorFrontEnabled(boolean enabled) {
        if(this.colorFrontState != null) {
            this.colorFrontState.setEnabled(enabled);
        }
    }

    public void setColorSensorBackEnabled(boolean enabled) {
        if(this.colorBackState != null) {
            this.colorBackState.setEnabled(enabled);
        }
    }

    public void setColorSensorBeaconEnabled(boolean enabled) {
        if(this.colorBeaconState != null) {
            this.colorBeaconState.setEnabled(enabled);
        }
    }

    public void setColorSensorParticleEnabled(boolean enabled) {
        if (this.colorParticleRightState != null) {
            this.colorParticleRightState.setEnabled(enabled);
        }
        if (this.colorParticleLeftState != null) {
            this.colorParticleLeftState.setEnabled(enabled);
        }
    }

    private boolean initBottomBackColorSensor() {
        boolean success = true;
        try
        {
            colorSensorBottomBack = (ModernRoboticsI2cColorSensor)hwMap.colorSensor.get("colorSensorBottomBack");
            colorSensorBottomBack.setI2cAddress(I2cAddr.create8bit(0x42));
            //colorSensorBottomBack.enableLed(false);

            if(colorSensorBottomBack.red() == 255) {
                colorSensorBottomBack = null;
                String error = " ** Bottom Color Sensor back Failed. " + " Stop and Restart !!!";
                telemetry.addData("02. Color: ", error);

                Log.e(this.getClass().getName(), error);
            }

            //colorBackState = new FtcI2cDeviceState(colorSensorBottomBack);
            telemetry.addData ("03.", "Color Bottom Back Initialized");
            logInfo("color sensor bottom back initialized");
        }
        catch (Exception e)
        {
            telemetry.addData("03.", "color sensor bottom back failed");
            Log.e(this.getClass().getSimpleName(), "Exception: " + e.getMessage());
            colorSensorBottomBack = null;
            success = false;
        }

        return success;
    }

    private boolean initLineDetectorODS() {
        boolean success = true;
        try
        {
            lineDetectorODS = hwMap.opticalDistanceSensor.get("lineDetector");
            telemetry.addData ("22.", "Line Detector ODS Initialized:" + lineDetectorODS.getLightDetected());
            logInfo("Line Detector ODS initialized");
        }
        catch (Exception e)
        {
            telemetry.addData("23.", "Line Detector ODS failed");
            Log.e(this.getClass().getSimpleName(), "Exception: " + e.getMessage());
            lineDetectorODS = null;
            success = false;
        }

        return success;
    }

    public double getLineDetectorLight() {
        if(lineDetectorODS != null) return lineDetectorODS.getLightDetected();

        return -1.0;
    }

    private boolean initRangeSensor() {
        boolean success = true;
        try {
            this.rangeSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensor");
            this.rangeSensor.getDistance(DistanceUnit.CM);

            telemetry.addData ("06", "Range sensor initialized ****");
            logInfo("range sensor initialized");
        }
        catch (Exception e)
        {
            telemetry.addData("06.", "Range sensor failed");
            Log.e(this.getClass().getSimpleName(), "Exception: " + e.getMessage());
            rangeSensor = null;
            success = false;
        }

        return success;
    }

    private boolean initShooterTouchSensor() {
        boolean success = true;
        try {
            shooterSwitch = hwMap.touchSensor.get("shooterSwitch");
            shooterSwitch.isPressed();

            logInfo("shooter touch initialized");
            telemetry.addData ("07", "shooter touch initialized ****");
        }
        catch (Exception ex)
        {
            RobotLog.e (ex.getLocalizedMessage ());
            shooterSwitch = null;
            telemetry.addData ("07", "shooter touch Init failed.");
            success = false;
        }

        return success;
    }

    private boolean initTensionTouchSensor() {
        boolean success = true;
        try {
            tensionSwitch = hwMap.touchSensor.get("tensionSwitch");
            tensionSwitch.isPressed();

            logInfo("Tension touch initialized");
            telemetry.addData ("08", "Tension touch initialized ****");
        }
        catch (Exception ex)
        {
            RobotLog.e (ex.getLocalizedMessage ());
            tensionSwitch = null;
            telemetry.addData ("08", "Tension touch Init failed.");
            success = false;
        }

        return success;
    }

    public double getRangeSensorDistance() {

        if(rangeSensor != null) {
            double distance = rangeSensor.getDistance(DistanceUnit.INCH);
            if(distance < 50.0) {
                previousRangeDistance = distance;
            }

            return previousRangeDistance;
        }
        return 0.0;
    }

    private boolean initBeaconColorSensor() {
        boolean success = true;
        try
        {
            colorSensorBeacon = (ModernRoboticsI2cColorSensor)hwMap.colorSensor.get("colorSensorBeacon");
            colorSensorBeacon.setI2cAddress(I2cAddr.create8bit(0x44));

            colorSensorBeacon.enableLed(false);

            if(colorSensorBeacon.red() == 255) {
                //colorSensorBeacon = null;
                String error = " ** Left Color Sensor Failed. " + " Stop and Restart !!!";
                telemetry.addData("04. Color: ", error);

                logInfo(error);
            }

            //colorBeaconState = new FtcI2cDeviceState(colorSensorBeacon);
            telemetry.addData ("04.", "Beacon Color Initialized");
            logInfo("color sensor left initialized");
        }
        catch (Exception e)
        {
            telemetry.addData("04.", "color sensor left failed");
            logInfo("Exception: " + e.getMessage());
            colorSensorBeacon = null;
            success = false;
        }

        return success;
    }

    private boolean initColorSensorParticleRight() {

        boolean success = true;
        try
        {
            colorSensorParticleRight = (ModernRoboticsI2cColorSensor)hwMap.colorSensor.get("colorSensorParticleRight");
            colorSensorParticleRight.setI2cAddress(I2cAddr.create8bit(0x48));
            //colorSensorParticleRight.enableLed(false);

            if(colorSensorParticleRight.red() == 255) {
                //colorSensorBeacon = null;
                String error = " ** Particle Color Sensor Failed. " + " Stop and Restart !!!";
                telemetry.addData("016. Color: ", error);

                logInfo(error);
            }

            //colorParticleRightState = new FtcI2cDeviceState(colorSensorParticleRight);
            telemetry.addData ("016.", "Particle Sensor Color Initialized");
            logInfo("color sensor particle initialized");
        }
        catch (Exception e)
        {
            telemetry.addData("016.", "Particle color sensor failed");
            logInfo("Exception: " + e.getMessage());
            colorSensorParticleRight = null;
            success = false;
        }

        return success;
    }
    private boolean initColorSensorParticleLeft() {

        boolean success = true;
        try
        {
            colorSensorParticleLeft = (ModernRoboticsI2cColorSensor)hwMap.colorSensor.get("colorSensorParticleLeft");
            //colorSensorParticleRight.enableLed(false);

            if(colorSensorParticleLeft.red() == 255) {
                //colorSensorBeacon = null;
                String error = " ** Particle Color Sensor Left Failed. " + " Stop and Restart !!!";
                telemetry.addData("017. Color: ", error);

                logInfo(error);
            }

            //colorParticleLeftState = new FtcI2cDeviceState(colorSensorParticleLeft);
            telemetry.addData ("017.", "Particle Sensor Color Left Initialized");
            logInfo("color sensor particle Left initialized");
        }
        catch (Exception e)
        {
            telemetry.addData("017.", "Particle color sensor Left failed");
            logInfo("Exception: " + e.getMessage());
            colorSensorParticleLeft = null;
            success = false;
        }

        return success;
    }

    public void setPowerUsingEncoders(double leftPower, double rightPower) {
        this.leftDriveMotor.setMode
                (DcMotor.RunMode.RUN_USING_ENCODER);

        this.rightDriveMotor.setMode
                (DcMotor.RunMode.RUN_USING_ENCODER);

        this.leftDriveMotor.setPower(leftPower);
        this.rightDriveMotor.setPower(rightPower);
    }

    public boolean setPowerRunToPosition(double leftPower, double rightPower, int target) {
        this.leftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.leftDriveMotor.setPower(leftPower);
        this.rightDriveMotor.setPower(rightPower);

        this.leftDriveMotor.setTargetPosition(this.lastLeftMotorEncoderValue  + target);
        this.rightDriveMotor.setTargetPosition(this.lastRightMotorEncoderValue  + target);

        if(!this.leftDriveMotor.isBusy()) {
            return true;
        }

        return false;
    }

    public int getLeftMotorPosition() {
        return this.leftDriveMotor.getCurrentPosition();
    }

    public int getLeftMotorDistanceTraveled() {
        return this.leftDriveMotor.getCurrentPosition() - this.lastLeftMotorEncoderValue;
    }

    public boolean setRunToPositionWithRangeSensor(double speed, int target, double distance)
    {
        this.leftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.leftDriveMotor.setTargetPosition(this.lastLeftMotorEncoderValue  + target);
        this.rightDriveMotor.setTargetPosition(this.lastRightMotorEncoderValue  + target);

        // adjust relative speed based on distance error.
        double error = Range.clip(this.getRangeSensorDistance(), 2.0, 8.0) - distance;
        double steer = Range.clip(error * 0.1, -0.5, 0.5);

        double direction = 1;
        if(target < 0) {
            direction = -1.0;
        }

        double leftSpeed = speed - direction*steer;
        double rightSpeed = speed + direction*steer;

        setPower(leftSpeed, rightSpeed);

        if(!this.leftDriveMotor.isBusy() && !this.leftDriveMotor.isBusy()) {
            return true;
        }

        return false;
    }

    public void setPower(double leftPower, double rightPower) {
        this.leftDriveMotor.setPower(leftPower);
        this.rightDriveMotor.setPower(rightPower);
    }

    public void drive(double speed) {
        this.leftDriveMotor.setPower(speed);
        this.rightDriveMotor.setPower(speed);
    }

    public void encoderDrive(double speed, double distance) {
        this.leftDriveMotor.setMode
                (DcMotor.RunMode.RUN_TO_POSITION);

        this.rightDriveMotor.setMode
                (DcMotor.RunMode.RUN_TO_POSITION);

        this.leftDriveMotor.setPower(speed);
        this.rightDriveMotor.setPower(speed);

        int moveCounts = (int)(distance * COUNTS_PER_INCH);
        this.leftDriveMotor.setTargetPosition(this.lastLeftMotorEncoderValue + moveCounts);
        this.rightDriveMotor.setTargetPosition(this.lastRightMotorEncoderValue + moveCounts);
    }

    public void setPowerWithoutEncoders(double leftPower, double rightPower) {
        this.leftDriveMotor.setMode
                (DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.rightDriveMotor.setMode
                (DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.leftDriveMotor.setPower(leftPower);
        this.rightDriveMotor.setPower(rightPower);
    }

    public void resetMotorEncoders() {
        if(this.leftDriveMotor != null) {
            // Reset motor encoders to 0
            this.leftDriveMotor.setMode
                    (DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        if(this.rightDriveMotor != null) {
            this.rightDriveMotor.setMode
                    (DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void logInfo(String message) {
        Log.i(this.getClass().getSimpleName(), message);
    }

    public double getHeadingAngle(){
        if(imu == null)  return -0.0;

        double tempAng = getRawHeadingAngle() - headingOffset;
        if(tempAng > 180.0) {
            tempAng = tempAng - 360;
        }
        else if(tempAng < -180.0) {
            tempAng = tempAng + 360;
        }
        return tempAng;
    }

    public boolean detectWhiteTape() {
        if(this.colorSensorBottomFront != null) {
            if(this.colorSensorBottomFront.green() > 4
                    && this.colorSensorBottomFront.red() > 4 && this.colorSensorBottomFront.blue() >4) {
                return true;
            }
        }

        return false;
    }

    public boolean detectRedTape() {
        if(this.colorSensorBottomFront != null) {
            if(this.colorSensorBottomFront.red() > 3
                    && this.colorSensorBottomFront.red() > this.colorSensorBottomFront.blue()) {
                return true;
            }
        }

        return false;
    }

    public boolean detectBlueTape() {
        if(this.colorSensorBottomFront != null) {
            if(this.colorSensorBottomFront.blue() > 3
                    && this.colorSensorBottomFront.red() < this.colorSensorBottomFront.blue()) {
                return true;
            }
        }

        return false;
    }

    public boolean detectWhiteTapeAny() {
        return detectWhiteTape() || detectWhiteTape2();
    }


    public boolean detectWhiteTape2() {
        if(this.colorSensorBottomBack != null) {
            if(this.colorSensorBottomBack.green() > 4
                    && this.colorSensorBottomBack.red() > 4 && this.colorSensorBottomBack.blue() >4) {
                return true;
            }
        }

        return false;
    }

    public int getBottomColorRedLevel() {
        if(this.colorSensorBottomFront != null) {
            return colorSensorBottomFront.red();
        }

        return 0;
    }

    public boolean hasTargetAngleReached(double newTarget, int turnLeft) {
        double headingAngle = getHeadingAngle();
        boolean reached = Math.abs(headingAngle - newTarget) < HEADING_ANGLE_TOLERANCE;

        if(!reached) {
            if(turnLeft>0) {
                //if(newTarget*headingAngle >0) {
                reached = headingAngle >= newTarget;
                //}
            } else {
                //if(newTarget*headingAngle >0) {
                reached = headingAngle <= newTarget;
                //}
            }
        }

        if(reached) {
            Log.i(this.getClass().getSimpleName(),
                    "Heading:" + String.format("%3.2f", headingAngle) + " Target: " + newTarget);
        }
        return reached;
    }

    public boolean hasLeftMotorTargetReached(int target) {
        return Math.abs(this.leftDriveMotor.getCurrentPosition() - lastLeftMotorEncoderValue)
                >= Math.abs(target);
    }

    public BeaconColor getBeaconColorSensor() {
        if(this.colorSensorBeacon == null) return BeaconColor.BAD_SENSOR;

        int blueLevel = this.colorSensorBeacon.blue();
        int redLevel = this.colorSensorBeacon.red();

        if(redLevel >= 1 && redLevel > blueLevel) {
            return BeaconColor.RED;
        }

        if(blueLevel >= 1 && blueLevel > redLevel) {
            return BeaconColor.BLUE;
        }

        return BeaconColor.UNKNOWN;
    }

    public Boolean isLeftSensorRedBeacon() {
        if(this.colorSensorBeacon == null) return null;

        int blueLevel = this.colorSensorBeacon.blue();
        int redLevel = this.colorSensorBeacon.red();

        return redLevel >= 1 && redLevel > blueLevel;
    }

    public Boolean isLeftSensorBlueBeacon() {
        if(this.colorSensorBeacon == null) return null;

        int blueLevel = this.colorSensorBeacon.blue();
        int redLevel = this.colorSensorBeacon.red();

        return redLevel >= 1 && redLevel > blueLevel;
    }

    public Boolean isParticleRed() {
        if(this.colorSensorParticleRight == null) return null;

        int blueLevel = this.colorSensorParticleRight.blue();
        int redLevel = this.colorSensorParticleRight.red();

        boolean rightParticleRed = (redLevel - blueLevel) >=4;
        if(this.colorSensorParticleLeft == null) return rightParticleRed;

        blueLevel = this.colorSensorParticleLeft.blue();
        redLevel = this.colorSensorParticleLeft.red();

        boolean leftParticleRed = (redLevel - blueLevel) >=4;
        return (leftParticleRed || rightParticleRed);
    }

    public Boolean isParticleBlue() {
        if(this.colorSensorParticleRight == null) return null;

        int blueLevel = this.colorSensorParticleRight.blue();
        int redLevel = this.colorSensorParticleRight.red();

        boolean rightParticleBlue = (blueLevel - redLevel) >=4;
        if(this.colorSensorParticleLeft == null) return rightParticleBlue;

        blueLevel = this.colorSensorParticleLeft.blue();
        redLevel = this.colorSensorParticleLeft.red();

        boolean leftParticleBlue = (blueLevel - redLevel) >=4;
        return (leftParticleBlue || rightParticleBlue);
    }

    public void extendLift(double power) {
        if(lifterMotor != null) {
            lifterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lifterMotor.setPower(Range.clip(power, -1.0, 1.0));
        }
    }

    double getRawHeadingAngle() {
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        return AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
    }

    public boolean gyroDrive ( double speed,
                               double distance,
                               double angle)
    {

        // Determine new target position, and pass to motor controller
        int moveCounts = (int)(distance * COUNTS_PER_INCH);
        // start motion.
        speed = Range.clip(Math.abs(speed), 0.0, 1.0);

        // adjust relative speed based on heading error.
        double error = getError(angle);
        double steer = getSteer(error, P_DRIVE_COEFF);

        // if driving in reverse, the motor correction also needs to be reversed
        if (distance < 0)
            steer *= -1.0;

        double leftSpeed = speed - steer;
        double rightSpeed = speed + steer;

        // Normalize speeds if any one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0)
        {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        this.leftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setPower(leftSpeed, rightSpeed);

        this.leftDriveMotor.setTargetPosition(this.lastLeftMotorEncoderValue  + moveCounts);
        this.rightDriveMotor.setTargetPosition(this.lastRightMotorEncoderValue  + moveCounts);

        if(!this.leftDriveMotor.isBusy() && !this.leftDriveMotor.isBusy()) {
            return true;
        }

        return false;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - getHeadingAngle();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    public boolean gyroTurn(double speed, double angle, double PCoeff) {
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        double error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        setPower(leftSpeed, rightSpeed);

        logInfo("error: " + error + " | " + leftSpeed + " | " + rightSpeed);
        return onTarget;
    }

    public double adjustAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }

    public boolean fastTurn(double targetAngle) {
        return turnP(targetAngle, 0.5, 0.02);
    }

    public boolean mediumTurn(double targetAngle) {
        return turnP(targetAngle, 0.25, 0.02);
    }

    public boolean slowTurn(double targetAngle) {
        return turnP(targetAngle, 0.10, 0.025);
    }

    public boolean slowTurnBlue(double targetAngle) {
        return turnP(targetAngle, 0.10, 0.03);
    }

    public boolean meduimTurnBlue(double targetAngle) {
        return turnP(targetAngle, -0.3, 0.02);
    }

    public boolean turn(double targetAngle, double speed) {
        return turnP(targetAngle, speed, 0.01);
    }

    public boolean turnP(double targetAngle, double speed, double kp) {
        double error;
        double power;
        double modifier = speed<0?-1:1;

        speed = speed*SPEED_GEAR_RATIO_FACTOR;
        error = getError(targetAngle);
        double tolerence = speed < 0.35?1.0:3.0;
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

        //power = Range.clip(power, -Math.abs(speed), +Math.abs(speed));
        setPower(-modifier*power, modifier*power);

        return false;
    }

    public String getColorSensorBottomFrontDisplay() {
        return "FRC: " + getColorSensorInfo(this.colorSensorBottomFront);
    }

    public int getColorSensorBottomFrontColor() {
        return this.colorSensorBottomFront!=null?
                this.colorSensorBottomFront.red() + this.colorSensorBottomFront.blue() + this.colorSensorBottomFront.green():0;
    }

    public int getColorSensorBottomBackColor() {
        return this.colorSensorBottomBack!=null?
                this.colorSensorBottomBack.red() + this.colorSensorBottomBack.blue() + this.colorSensorBottomBack.green():0;
    }

    public int getColorSensorBeaconColor() {
        return this.colorSensorBeacon!=null?
                this.colorSensorBeacon.red() + this.colorSensorBeacon.blue() + this.colorSensorBeacon.green():0;
    }

    public String getColorSensorBottomBackDisplay() {
        return "BAC: " + getColorSensorInfo(this.colorSensorBottomBack);
    }
    public String getBeaconColorSensorDisplay() {
        return "BEC: " + getColorSensorInfo(this.colorSensorBeacon);
    }

    public String getParticleColorSensorDisplay() {
        return " R_PAC: " + getColorSensorInfo(this.colorSensorParticleRight)
                + " | L_PAC: " + getColorSensorInfo(this.colorSensorParticleLeft);
    }

    private String getColorSensorInfo(ColorSensor color) {
        String info =" == Bad Color Sensor!!! ";

        if(color != null) {
            info = " RGB: (" +
                    color.red() + "," + color.green() + "," + color.blue() + ")";
        }

        return info;
    }

    public double getBatteryVoltage() {
        if(motorController != null) {
            return motorController.getVoltage();
        }

        return 0.0;
    }

    public void resetEncoders() {
        this.lastLeftMotorEncoderValue = leftDriveMotor.getCurrentPosition();
        this.lastRightMotorEncoderValue = rightDriveMotor.getCurrentPosition();
    }
}
