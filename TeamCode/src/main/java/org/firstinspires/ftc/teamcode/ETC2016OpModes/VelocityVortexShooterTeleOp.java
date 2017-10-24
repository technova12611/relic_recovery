package org.firstinspires.ftc.teamcode.ETC2016OpModes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ETCHardware.ETC2016Robot;

/**
 * Created by FTC 9915
 */
@TeleOp(name = "Test Shooter and Sensors", group = "Test")
@Disabled
public class VelocityVortexShooterTeleOp extends ETCBaseOpMode {

    public VelocityVortexShooterTeleOp() {
    }

    protected ETC2016Robot robot = new ETC2016Robot();

    private static final int COLLECTOR_MOTOR_STOP = 0;
    private static final int COLLECTOR_MOTOR_FORWARD = 1;
    private static final int COLLECTOR_MOTOR_BACKWARD = 2;

    //Initialize to stop.
    private int collectorMotorState = COLLECTOR_MOTOR_STOP;

    private AllianceColor allianceColor = AllianceColor.RED;

    boolean waitForReset = false;
    int count = 0;
    ElapsedTime matchTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime particleTimer = null;

    double particleDetectedTime = 0.0;

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry, true);
        logData("00 OpMode: ", " --- TeleOps (Driver Control) Initialized ---");
    }

    public void init_loop() {

        super.init_loop();
        boolean shooterInitSuccess = this.robot.ballShooter.init_loop(true, false);

        //updateGamepadTelemetry();
        telemetry.addLine("Shooter Status: " + shooterInitSuccess);
        telemetry.update();
    }

    @Override
    public void start() {
        super.start();
        robot.ballShooter.start();
        robot.leftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.robot.colorSensorParticleRight.enableLed(true);
        this.robot.setColorSensorParticleEnabled(true);

        matchTimer.reset();

        logData("100", " === Velocity TeleOps Started ===");
    }

//**************************************************************************************************

    @Override
    public void loop() {

//**************************************************************************************************
        count++;
        if(count%100 == 0) {
            count =0;
        }

        tankDrive();
        shootParticles();
        adjustTension();
        pushBeaconButton();
        moveStopper();

        collectParticles();

        if(gamepad2.start && gamepad2.left_bumper) {
            if(allianceColor == AllianceColor.RED) {
                allianceColor = AllianceColor.BLUE;
            } else {
                allianceColor = AllianceColor.RED;
            }
        }

        //*************************************************************************************************
        //Send telemetry data to the driver station.
        //updateGamepadTelemetry();

        displaySensorData();
        telemetry.update();
    }

    private void shootParticles() {

        if(robot.ballShooter != null) {

            robot.ballShooter.fire_loop();

            // when gamepad2 button a pressed
            if (gamepad2.a ) {
                robot.ballShooter.fire();
            }

            if(gamepad2.b) {
                robot.ballShooter.stop();
            }
        }
    }

    private void adjustTension() {
        if(robot.ballShooter != null) {
            robot.ballShooter.adjustTension(scaleMotorPower(-1*gamepad2.right_stick_y));
        }
    }

    private void pushBeaconButton() {
        if(robot.buttonPusher != null) {
            if (gamepad2.x) {
                robot.buttonPusher.pushButton();
            } else if (gamepad2.y) {
                robot.buttonPusher.retract();
            } else if (gamepad2.right_bumper) {
                robot.buttonPusher.reset();
            }
        }
    }

    private void moveStopper() {
        if(robot.ballShooter != null) {
            if(Math.abs(gamepad2.left_stick_x) > 0.5) {
                robot.ballShooter.moveStopper(-1.0);
                //Log.i(this.getClass().getSimpleName(), "Open Stopper " + gamepad2.left_stick_x);
            } else {
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

            float leftDrivePower = modifier * scaleMotorPower(-1 * gamepad1.left_stick_y);
            float rightDrivePower = modifier * scaleMotorPower(-1 * gamepad1.right_stick_y);

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

        DcMotor motor = robot.collectorMotor;//!= null? robot.collectorMotor: robot.tensionMotor;
        if(motor != null) {

            switch (collectorMotorState) {
                case COLLECTOR_MOTOR_FORWARD:
                    motor.setPower(-1.0);

                    if(particleTimer!= null && particleTimer.time() > 750) {
                        collectorMotorState = COLLECTOR_MOTOR_STOP;
                    }
                    break;
                case COLLECTOR_MOTOR_BACKWARD:

                    if(getAllianceColor() == AllianceColor.RED && robot.isParticleBlue()) {
                        collectorMotorState = COLLECTOR_MOTOR_FORWARD;
                        particleTimer=new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
                        Log.i(this.getClass().getSimpleName(), matchTimer.time() + " Caught blue, reverse... ");
                    } else if(getAllianceColor() == AllianceColor.BLUE && robot.isParticleRed()) {
                        collectorMotorState = COLLECTOR_MOTOR_FORWARD;
                        particleTimer=new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
                        Log.i(this.getClass().getSimpleName(), matchTimer.time() + " Caught red, reverse... ");
                    }

                    if(robot.isParticleBlue() || robot.isParticleRed()) {
                        Log.i(this.getClass().getSimpleName(),
                                String.format(".1%f",matchTimer.time()) + robot.getParticleColorSensorDisplay());
                    }

                    if(((getAllianceColor() == AllianceColor.RED && robot.isParticleRed()) ||
                            (getAllianceColor() == AllianceColor.BLUE && robot.isParticleBlue())) ||
                            (matchTimer.time() - particleDetectedTime) < 1000.0 ) {
                        motor.setPower(0.95);
                        particleDetectedTime = matchTimer.time();
                        Log.i(this.getClass().getSimpleName(),
                                "Particle Color: " + robot.isParticleRed() + " | b: " + robot.isParticleBlue());
                    } else {
                        motor.setPower(0.40);
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
        super.stop();
        waitForReset = false;
        Log.i(this.getClass().getName(), "TeleOp Stopped");
        robot.ballShooter.moveTensionToZeroPosition();
    }

    private void logData(String name, String message) {
        if(count == 0) {
            telemetry.addData(name, message);
            Log.i(this.getClass().getSimpleName(), name + ":" + message);
        }
    }

    private void displaySensorData() {
        telemetry.addData("0", "Alliance Color: " + allianceColor);
        telemetry.addData("00", "Tension: " + robot.ballShooter.getTensionPosition());

        telemetry.addData("000c",robot.getColorSensorBottomFrontDisplay());
        telemetry.addData("000c",robot.getColorSensorBottomBackDisplay());
        telemetry.addData("000a", robot.getBeaconColorSensorDisplay());

        telemetry.addData("001", "Gyro: " + String.format("%.2f",robot.getHeadingAngle()));
        telemetry.addData("002", "Range: " + String.format("%.2f",robot.getRangeSensorDistance()));

        telemetry.addData("004", "Tension Switch: " + robot.ballShooter.isTensionSwitchPressed());
        telemetry.addData("005", "Shooter Switch: " + robot.ballShooter.isShooterSwitchPressed());

        telemetry.addData("006", "Left Drive:" + robot.leftDriveMotor.getCurrentPosition());
        telemetry.addData("007", "Right Drive:" + robot.leftDriveMotor.getCurrentPosition());

        telemetry.addData("008", "Push Switch:" + robot.pusherLimitSensor.getState());
        telemetry.addData("010", robot.getParticleColorSensorDisplay());
    }

    @Override
    public AllianceColor getAllianceColor() {
        return allianceColor;
    }
}

//**************************************************************************************************