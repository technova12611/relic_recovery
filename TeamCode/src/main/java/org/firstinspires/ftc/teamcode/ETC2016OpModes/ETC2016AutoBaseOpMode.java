package org.firstinspires.ftc.teamcode.ETC2016OpModes;

import android.media.SoundPool;
import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ETCHardware.ETC2016Robot;
import org.firstinspires.ftc.teamcode.ETCHardware.ETCConstants;


/**
 * for FTC 2016 Velocity Vortex Game
 */
public abstract class ETC2016AutoBaseOpMode extends ETCBaseOpMode {

    protected ETC2016Robot robot = new ETC2016Robot();
    protected ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    protected ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public static int RED_ALLIANCE = 1;
    public static int BLUE_ALLIANCE = -1;

    protected boolean isStateStart = true;
    protected ETC2016Robot.BeaconColor beaconColor;

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.57142857142;     // This is < 1.0 if geared UP (48/84)
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double     MOVE_TO_WHITE_TAPE_POWER    = ETCConstants.MOVE_TO_WHITE_TAPE_POWER;

    //---------------------------------------------------------------------------------
    //
    //              STATE MACHINE
    //
    protected final static int    START                                  = 0;

    protected int v_state = START;
    protected int prev_state = START;

    protected SoundPool mySound;
    protected int successSound;
    protected int failedSound;

    protected double previousSensorData = 0.0;
    protected boolean enableColorSensorRegister = false;

    protected double driveMotorPower = 0.0;

    protected boolean isDefensiveAllowed = true;

    protected boolean madeAdjustment = false;

    protected int betweenBeaconStartEncoderCount = 0;
    protected int distanceBetweenBeacons = (int) (COUNTS_PER_INCH * 36.0);

    protected void preInit() {
        this.msStuckDetectInit = 10000;
    }

    @Override
    public void init() {
        elapsedTime.reset();
        telemetry.addData("00 OpMode: ",
                (getAlliance() == RED_ALLIANCE? "*** RED: ": "--- BLUE: ") +
                 "--- Autonomous ---");

        robot.hasGyroSensor = initGyro();
        boolean success = robot.init(hardwareMap, telemetry, true);

//        mySound = new SoundPool(1, AudioManager.STREAM_MUSIC, 0);
//        successSound = getSuccessSound();
//        failedSound = getFailedSound();
//
//        delay(200);
//
//        if(!success) {
//            mySound.play(successSound, 1, 1, 1, 0, 1);
//            if(robot.dim != null) {
//                robot.dim.setLED(1, true);
//            }
//        } else {
//            mySound.play(failedSound, 1, 1, 1, 0, 1);
//            if(robot.dim != null) {
//                robot.dim.setLED(0, true);
//            }
//        }

        robot.resetMotorEncoders();

        telemetry.update();
        elapsedTime.reset();
    }

    protected boolean initGyro() {
        return true;
    }

    @Override
    public void init_loop() {
//        if(elapsedTime.time() > 10000) {
//            updateSensorInfoInTelemetry();
//        }
//
//        telemetry.update();
        //this.robot.colorSensorBottomBack.enableLed(true);
    }

//    protected int getSuccessSound() {
//        return mySound.load(hardwareMap.appContext, R.raw.cello, 1);
//    }
//
//    protected int getFailedSound() {
//        return mySound.load(hardwareMap.appContext, R.raw.siren, 1);
//    }

    protected void logSensorInfo(boolean begin) {
        String sensorInfo = getSensorInfo(begin, true);
        Log.i(this.getClass().getSimpleName(), sensorInfo);
        if(!begin) telemetry.addLine("Sensor Data: ").addData(" ", sensorInfo);
    }

    protected void logSensorInfoOnChange() {
        double currentSensorInfo = robot.getHeadingAngle()+
                robot.getRangeSensorDistance()+
                robot.getColorSensorBeaconColor() +
                robot.getColorSensorBottomFrontColor()+
                robot.getColorSensorBottomBackColor() +
                robot.getLineDetectorLight();

        if(!String.format("%.2f", currentSensorInfo).equals(String.format("%.2f", previousSensorData))) {
            logSensorInfo(true);
            previousSensorData = currentSensorInfo;
        }
    }

    protected String getSensorInfo(boolean begin, boolean includeState) {
        String sensorInfo = "Error";
        try {
            sensorInfo = (includeState?(begin ? "--- " : "*** ") + "State: " + v_state:"") +
                    " | elapsed: " + String.format("%.1f", elapsedTime.time()) +
                    " | Motor: L=" + (robot.leftDriveMotor.getCurrentPosition() - robot.lastLeftMotorEncoderValue) +
                    " R= " + (robot.rightDriveMotor.getCurrentPosition() - robot.lastRightMotorEncoderValue) +
                    " | Angle:" + String.format("%.1f", robot.getHeadingAngle()) +
                    " | Range Sensor: " + String.format("%.2f", robot.getRangeSensorDistance()) +
                    " | " + robot.getColorSensorBottomFrontDisplay() +
                    " | " + robot.getColorSensorBottomBackDisplay() +
                    " | " + robot.getBeaconColorSensorDisplay() +
                    " | Vol: " + String.format("%.2f", robot.getBatteryVoltage()) ;

        }
        catch(Exception e) {
            sensorInfo = e.getMessage();
        }

        return sensorInfo;
    }

    protected boolean isDefensivePlay() {
        return false;
    }

    protected boolean parkAtCenter(){return true;}

    protected int getNumOfParticles() { return 2;}

    protected void updateSensorInfoInTelemetry() {
        String sensorInfo = "Error";
        try {
            sensorInfo =
                    " angle:" + String.format("%.1f", robot.getHeadingAngle()) +
                            " | " + robot.getColorSensorBottomFrontDisplay() +
                            " | " + robot.getColorSensorBottomBackDisplay() +
                            " | " + robot.getBeaconColorSensorDisplay() +
                            " | Range Sensor: " + robot.getRangeSensorDistance()
                            + " | Vol: " + robot.getBatteryVoltage();
            ;
        } catch(Exception e) {
            sensorInfo = e.getMessage();
        }
        telemetry.addLine("Sensor Data: ").addData(" ", sensorInfo);
    }

    protected void goToNextState() {
        stateTransition();
        prev_state = v_state;
        v_state++;
        isStateStart = true;
    }

    protected void goToState(int nextState) {
        stateTransition();
        prev_state = v_state;
        v_state = nextState;
    }

    protected void stateTransition() {
        robot.setPower(0.0,0.0);
        logSensorInfo(false);
        robot.lastLeftMotorEncoderValue = robot.leftDriveMotor.getCurrentPosition();
        robot.lastRightMotorEncoderValue = robot.rightDriveMotor.getCurrentPosition();
        robot.stateBeginHeadingAngle = robot.getHeadingAngle();
        robot.stateBeginDistance = robot.getRangeSensorDistance();

        robot.leftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.ballShooter.stopTensionMotor();
        robot.ballShooter.moveStopper(1.0);
        timer.reset();
    }

    protected void resetState() {
        robot.setPower(0.0,0.0);
        logSensorInfo(true);
        robot.lastLeftMotorEncoderValue = robot.leftDriveMotor.getCurrentPosition();
        robot.lastRightMotorEncoderValue = robot.rightDriveMotor.getCurrentPosition();

        robot.leftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    protected int getAlliance() {
        return RED_ALLIANCE;
    }

    @Override public void stop() {
        super.stop();
        robot.ballShooter.moveTensionToZeroPosition();
    }

    protected void logInfo(String message) {
        Log.i(this.getClass().getSimpleName(), String.format("%.1f", getRuntime()) + ": " + message);
    }
}
