package org.firstinspires.ftc.teamcode.ETC2016OpModes;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ETCHardware.ETC2016Robot;
import org.firstinspires.ftc.teamcode.ETCHardware.ETCConstants;

/**
 * Created by Robotics 6 on 10/23/2016.
 */

public abstract class VelocityVortexTeleOp extends ETCBaseOpMode {

    public VelocityVortexTeleOp() {
    }

    protected ETC2016Robot robot = new ETC2016Robot();

    private static final int COLLECTOR_MOTOR_STOP = 0;
    private static final int COLLECTOR_MOTOR_FORWARD = 1;
    private static final int COLLECTOR_MOTOR_BACKWARD = 2;


    private static final int TENTION_STATE_STOP = 0;

    private static final int TENTION_STATE_ZERO = 1;
    private static final int TENTION_STATE_LOW = 2;
    private static final int TENTION_STATE_MEDIUM = 3;
    private static final int TENTION_STATE_HIGH = 4;
    private static final int TENTION_STATE_VERY_HIGH = 5;

    private static final int TENTION_STATE_MANUAL = 10;


    //Initialize to stop.
    private int collectorMotorState = COLLECTOR_MOTOR_STOP;

    boolean particleDetectionEnabled = true;
    boolean waitForReset = false;

    int tension_state = TENTION_STATE_STOP;

    boolean modifierState = false;

    ElapsedTime gameTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    ElapsedTime matchTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime particleTimer = null;

    double particleDetectedTime = 0.0;

    private Boolean teleOpsStart = null;

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry, false);
        logData("00 OpMode: ", " --- TeleOps " + (getAllianceColor() == AllianceColor.RED? "RED":"BLUE") +" Initialized ---");
    }

    public void init_loop() {

        super.init_loop();
        boolean shooterInitSuccess = this.robot.ballShooter.init_loop(false, false);

        telemetry.addLine("Shooter init Status: " + shooterInitSuccess);
        telemetry.addLine("Tension: " + this.robot.ballShooter.getTensionPosition());
        telemetry.addLine("Shooter Switch: " + this.robot.ballShooter.isShooterSwitchPressed());
        telemetry.addLine("Tension switch: " + this.robot.ballShooter.isTensionSwitchPressed());

        //updateGamepadTelemetry();
        telemetry.update();
    }

    @Override
    public void start() {
        super.start();
        robot.ballShooter.start();
        robot.leftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.robot.setColorSensorParticleEnabled(true);

        gameTimer.reset();
        robot.ballShooter.resetGameTimer();

        logData("100", " === Velocity TeleOps Started ===");
    }

//**************************************************************************************************

    @Override
    public void loop() {

//**************************************************************************************************

        //Log.i(this.getClass().getSimpleName(), "Loop Starts ....");

        if(gamepad1.left_trigger > 0.5) {
            modifierState = true;
        }

        if(gamepad1.right_trigger > 0.5) {
            modifierState = false;
        }

        if(modifierState) {
            tankDriveSlowMode();
        } else {
            tankDrive3();
        }

        //Log.i(this.getClass().getSimpleName(), "Shoot Particles");

        shootParticles();
        if(gameTimer.time() > 3000) {
            //Log.i(this.getClass().getSimpleName(), "Adjust tension");
            adjustTension();
        } else {
            robot.ballShooter.stopTensionMotor();
        }

        //Log.i(this.getClass().getSimpleName(), "Other actions");

        pushBeaconButton();
        moveStopper();
        releaseCapFork();
        liftCapBall();
        collectParticles();

        //*************************************************************************************************
        //Send telemetry data to the driver station.
        //updateGamepadTelemetry();

        displaySensorData();
        telemetry.update();

        if(teleOpsStart == null &&
                Math.abs(gamepad1.left_stick_y) > 0.25 && Math.abs(gamepad1.right_stick_y) > 0.25) {
            teleOpsStart = true;
            this.robot.ballShooter.resetGameTimer();
            gameTimer.reset();

            Log.i(this.getClass().getSimpleName(), "Driver Control Started ....");
        }

        //Log.i(this.getClass().getSimpleName(), "All Done ...");
    }

    private void shootParticles() {

        if(robot.ballShooter != null) {

            if(gamepad2.right_bumper && gamepad2.a ) {
                //robot.ballShooter.unstuck();
            }
            else if(gamepad2.b) {
                robot.ballShooter.stop();
            }

            // when gamepad2 button a pressed
            else if (gamepad2.a ) {
                if(gamepad2.left_bumper) {
                    robot.ballShooter.manualReload();
                }
                else {
                    if(robot.ballShooter.isReadyToFire()) {
                        Log.i(this.getClass().getSimpleName(), "Batter Voltage:" +
                                String.format("%.2f",robot.getBatteryVoltage()));
                    }

                    robot.ballShooter.fire();
                }
            }

            robot.ballShooter.fire_loop();
        }
    }

    private void adjustTension() {
        if(robot.ballShooter != null) {
            if(Math.abs(gamepad2.right_stick_y) > 0.05) {
                tension_state = TENTION_STATE_MANUAL;
            }
            else if (gamepad2.x && gamepad2.left_bumper) {
                tension_state = TENTION_STATE_LOW;
            }
            else if (gamepad2.y && gamepad2.left_bumper) {
                tension_state = TENTION_STATE_VERY_HIGH;
            }
            else if (gamepad2.x) {
                tension_state = TENTION_STATE_MEDIUM;
            }
            else if (gamepad2.y) {
                tension_state = TENTION_STATE_HIGH;
            }

            switch(tension_state) {

                case TENTION_STATE_ZERO:
                    if(robot.ballShooter.moveTensionToZeroPosition()) {
                        tension_state = TENTION_STATE_STOP;
                    }
                    break;
                case TENTION_STATE_LOW:
                    if(robot.ballShooter.moveTensionToLowPosition()) {
                        tension_state = TENTION_STATE_STOP;
                    }
                    break;

                case TENTION_STATE_MEDIUM:
                    if(robot.ballShooter.moveTensionToMediumPosition()) {
                        tension_state = TENTION_STATE_STOP;
                    }
                    break;

                case TENTION_STATE_HIGH:
                    if(robot.ballShooter.moveTensionToHighPosition()) {
                        tension_state = TENTION_STATE_STOP;
                    }
                    break;
                case TENTION_STATE_VERY_HIGH:
                    if(robot.ballShooter.moveTensionToVeryHighPosition()) {
                        tension_state = TENTION_STATE_STOP;
                    }
                    break;
                case TENTION_STATE_MANUAL:
                    robot.ballShooter.adjustTension(scaleMotorPower(-1 * gamepad2.right_stick_y));
                    if(gamepad2.right_stick_y == 0) {
                        tension_state = TENTION_STATE_STOP;
                    }
                    break;
            }

            if(tension_state == TENTION_STATE_STOP) {
                robot.ballShooter.stopTensionMotor();
            }
        }
    }

    private void pushBeaconButton() {
        if(robot.buttonPusher != null) {
            if (gamepad1.x) {
                robot.buttonPusher.pushButton();
            } else if (gamepad1.y) {
                robot.buttonPusher.retract();
            }
        }
    }

    private void releaseCapFork() {
        if(gamepad1.left_bumper && gamepad1.right_bumper)
        {
            robot.releaseServo.setPosition(0.65);
        }
    }

    private void liftCapBall() {
        if(this.robot.lifterMotor != null) {
            if( (this.robot.lifterMotor.getCurrentPosition() >
                    (int)(7.85* ETCConstants.CPR_LIFT_MOTOR) && gamepad2.left_stick_y < 0) ||
                    ((this.robot.lifterMotor.getCurrentPosition() < -10
                            && !gamepad2.left_bumper)&& gamepad2.left_stick_y >0)) {
                this.robot.lifterMotor.setPower(0.0);
            }
            else {
                if (!gamepad2.right_bumper && Math.abs(gamepad2.left_stick_x) < 0.25) {

                    if (gamepad2.left_stick_y > 0.0) {
                        this.robot.extendLift(Range.clip(-1 * gamepad2.left_stick_y, -0.60, 0.0));
                    } else {
                        this.robot.extendLift(-1 * gamepad2.left_stick_y);
                    }
                }
            }
        }
    }

    private void moveStopper() {
        if(robot.ballShooter != null) {
            if (gamepad2.left_stick_x > 0.8) {
                robot.ballShooter.moveStopper(1.0);
            }
            else if(gamepad2.left_stick_x < -0.8) {
                robot.ballShooter.moveStopper(-1.0);
            }
            else {
                robot.ballShooter.moveStopper(0.0);
            }
        }
    }

    private void tankDrive() {

        if(robot.leftDriveMotor != null && robot.rightDriveMotor != null) {

            int modifier = 1;

            if (gamepad1.right_bumper) {
                modifier = -1;
            }

            float leftDrivePower = modifier * scaleMotorPower(-gamepad1.left_stick_y);
            float rightDrivePower = modifier * scaleMotorPower(-gamepad1.right_stick_y);

            robot.leftDriveMotor.setPower(modifier > 0 ? leftDrivePower : rightDrivePower);
            robot.rightDriveMotor.setPower(modifier > 0 ? rightDrivePower : leftDrivePower);
        }
    }

    private void tankDriveSlowMode() {

        if(robot.leftDriveMotor != null && robot.rightDriveMotor != null) {

            int modifier = 1;

            if (gamepad1.right_bumper) {
                modifier = -1;
            }

            float leftDrivePower = modifier * scaleMotorPowerSlowMode(-gamepad1.left_stick_y);
            float rightDrivePower = modifier * scaleMotorPowerSlowMode(-gamepad1.right_stick_y);

            if(leftDrivePower*rightDrivePower <= 0 ) {
                leftDrivePower = (float)Range.clip(leftDrivePower, -0.3,0.3);
                rightDrivePower = (float)Range.clip(rightDrivePower, -0.3,0.3);
            }
            robot.leftDriveMotor.setPower(modifier > 0 ? leftDrivePower : rightDrivePower);
            robot.rightDriveMotor.setPower(modifier > 0 ? rightDrivePower : leftDrivePower);
        }
    }

    private void tankDrive2() {

        if(robot.leftDriveMotor != null && robot.rightDriveMotor != null) {

            int modifier = 1;
            float modifier2 = 1.0f;

            if (gamepad1.right_bumper) {
                modifier = -1;
            }

            if(gamepad1.left_trigger > 0.5) {
                modifierState = true;
            }

            if(gamepad1.right_trigger > 0.5) {
                modifierState = false;
            }

            if(modifierState) {
                modifier2 = 0.32f;

                if(Math.abs(gamepad1.left_stick_y) > 0.80) {
                    modifier2 = 0.40f;
                }

                if(Math.abs(gamepad1.left_stick_y) < 0.5) {
                    modifier2 = 0.60f;
                }
            }

            float leftDrivePower = modifier * modifier2 * Range.clip(
                                     -(gamepad1.left_stick_y*Math.abs(gamepad1.left_stick_y)), -1,1);
            float rightDrivePower = modifier * modifier2 * Range.clip(
                    -(gamepad1.right_stick_y*Math.abs(gamepad1.right_stick_y)), -1,1);

            robot.leftDriveMotor.setPower(modifier > 0 ? leftDrivePower : rightDrivePower);
            robot.rightDriveMotor.setPower(modifier > 0 ? rightDrivePower : leftDrivePower);
        }
    }

    private void tankDrive3() {

        if(robot.leftDriveMotor != null && robot.rightDriveMotor != null) {

            int modifier = 1;
            float modifier2 = 1;

            if (gamepad1.right_bumper) {
                modifier = -1;
            }

            if(gamepad1.left_trigger > 0.5) {
                modifierState = true;
            }

            if(gamepad1.right_trigger > 0.5) {
                modifierState = false;
            }

            if(modifierState) {
                modifier2 = 0.25f;
            }

            float leftDrivePower = modifier * modifier2 * Range.clip(
                    -(gamepad1.left_stick_y*gamepad1.left_stick_y*gamepad1.left_stick_y), -1,1);
            float rightDrivePower = modifier * modifier2 * Range.clip(
                    -(gamepad1.right_stick_y*gamepad1.right_stick_y*gamepad1.right_stick_y), -1,1);

            robot.leftDriveMotor.setPower(modifier > 0 ? leftDrivePower : rightDrivePower);
            robot.rightDriveMotor.setPower(modifier > 0 ? rightDrivePower : leftDrivePower);
        }
    }

    private void collectParticles() {
        // Map Trigger to control collector motor: COLLECT, SWEEP and STOP.
        // run to collect
        //******************************************************************************************
        if (gamepad2.right_trigger > 0.5) {
            collectorMotorState = COLLECTOR_MOTOR_FORWARD;
        }
        // run to sweep
        else if (gamepad2.left_trigger > 0.5) {
            collectorMotorState = COLLECTOR_MOTOR_BACKWARD;

        } else if (gamepad2.left_bumper) {
            collectorMotorState = COLLECTOR_MOTOR_STOP;
        }

        if(gamepad2.left_bumper && gamepad2.right_bumper) {
            particleDetectionEnabled = false;
        }
        DcMotor motor = robot.collectorMotor;//!= null? robot.collectorMotor: robot.tensionMotor;
        if(motor != null) {
            switch (collectorMotorState) {
                case COLLECTOR_MOTOR_FORWARD:

                    motor.setPower(-1.0);
                    if(particleTimer!= null && particleTimer.time() > 1000) {
                        collectorMotorState = COLLECTOR_MOTOR_STOP;
                    }
                    break;
                case COLLECTOR_MOTOR_BACKWARD:
                    if(particleDetectionEnabled) {
                        if (getAllianceColor() == AllianceColor.RED && robot.isParticleBlue()) {
                            collectorMotorState = COLLECTOR_MOTOR_FORWARD;
                            particleTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
                            Log.i(this.getClass().getSimpleName(), matchTimer.time() + " Caught blue, reverse... ");
                        } else if (getAllianceColor() == AllianceColor.BLUE && robot.isParticleRed()) {
                            collectorMotorState = COLLECTOR_MOTOR_FORWARD;
                            particleTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
                            Log.i(this.getClass().getSimpleName(), matchTimer.time() + " Caught red, reverse... ");
                        }

                        if ( (robot.isParticleBlue() || robot.isParticleRed()) &&
                                (matchTimer.time() - particleDetectedTime) > 1000){
                            Log.i(this.getClass().getSimpleName(),
                                    String.format("%.1f", matchTimer.time()) + robot.getParticleColorSensorDisplay());
                        }

                        if((getAllianceColor() == AllianceColor.RED && robot.isParticleRed()) ||
                                (getAllianceColor() == AllianceColor.BLUE && robot.isParticleBlue())) {
                            particleDetectedTime = matchTimer.time();
                        }

                        if( (matchTimer.time() - particleDetectedTime) < 1000.0 ) {
                            motor.setPower(0.95);
                        }
                        else {
                            motor.setPower(0.75);
                        }
                    } else {
                        motor.setPower(0.95);
                    }

                    break;

                default:
                    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    motor.setPower(0.0);
                    particleTimer = null;
            }
        }
    }

    @Override
    public void stop() {
        waitForReset = false;
        Log.i(this.getClass().getSimpleName(), "Total Shots Fired:" + this.robot.ballShooter.getNumOfShots());
        Log.i(this.getClass().getSimpleName(), "TeleOp Stopped");
    }

    private void logData(String name, String message) {
        telemetry.addData(name, message);
        Log.i(this.getClass().getSimpleName(),
                String.format("%.1f", gameTimer.time()) + " | " + name + ":" + message);
    }

    private void displaySensorData() {
        telemetry.addData("00", "Tension: " + robot.ballShooter.getTensionPosition());
        if(robot.lifterMotor != null) {
            telemetry.addData("006", "Lifter Position: " + robot.lifterMotor.getCurrentPosition());
        }

        telemetry.addData("01", "# of Shots Fired: " + robot.ballShooter.getNumOfShots());
        telemetry.addData("04", "Tension Switch: " + robot.ballShooter.isTensionSwitchPressed());
        telemetry.addData("05", "Shooter Switch: " + robot.ballShooter.isShooterSwitchPressed());
        telemetry.addData("06", "Particle Detection: " +particleDetectionEnabled);
        telemetry.addData("07", "Particle Color: " + robot.getParticleColorSensorDisplay());

        telemetry.addData("10", "Shooter Cycle Time: " + String.format("%.1f",robot.ballShooter.getCycleTime()));
        telemetry.addData("11", "Shooter Overshoot: " + robot.ballShooter.isOverShoot());
        telemetry.addData("09", "Vol: " + String.format("%.1f",robot.getBatteryVoltage()));

        if(collectorMotorState == COLLECTOR_MOTOR_BACKWARD) {
            telemetry.addData("08", "Collector State: COLLECT");
        }
        else if(collectorMotorState == COLLECTOR_MOTOR_STOP) {
            telemetry.addData("08", "Collector State: STOP");
        }
        else if(collectorMotorState == COLLECTOR_MOTOR_FORWARD) {
            telemetry.addData("08", "Collector State: REVERSE");
        }

//        telemetry.addData("000c",robot.getColorSensorBottomFrontDisplay());
//        telemetry.addData("000c",robot.getColorSensorBottomBackDisplay());
//        telemetry.addData("000a", robot.getBeaconColorSensorDisplay());

//        telemetry.addData("001", "Gyro: " + String.format("%.2f",robot.getHeadingAngle()));
//        telemetry.addData("002", "Range: " + String.format("%.2f",robot.getRangeSensorDistance()));


//        telemetry.addData("006", "Left Drive:" + robot.leftDriveMotor.getCurrentPosition());
//        telemetry.addData("007", "Right Drive:" + robot.leftDriveMotor.getCurrentPosition());
//
//        telemetry.addData("008", "Push Switch:" + robot.pusherLimitSensor.getState());
    }
}

//**************************************************************************************************
