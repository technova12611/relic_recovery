package org.firstinspires.ftc.teamcode.TechNova2017;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import java.util.concurrent.TimeUnit;

import static org.firstinspires.ftc.teamcode.TechNova2017.RobotInfo.DISTANCE_SENSOR_UPRIGHT_POSITION;

/**
 * Provide the basic functions for robot to perform run to distance and turn to angle
 * and other sensor related control algorithms
 */
public abstract class TileRunnerAbstract extends LinearOpMode {
    TileRunnerRobot robot = null;

    public void driveForwardInches(double inches, double power) {
        driveDirectionInches(Math.PI,inches, power);
    }

    public void driveBackwardInches(double inches, double power) {
        driveDirectionInches(0,inches, power);
    }

    protected void driveDirectionInches(double directionRadians, double inches, double power)
    {
        robot.setEncoderDrivePower(power);
        robot.encoderDriveInches(directionRadians, inches);

        while (opModeIsActive() && robot.driveMotorsBusy())
        {
            idle();
        }

        robot.stopDriveMotors();
        robot.resetDriveMotorModes();
        robot.clearEncoderDrivePower();
    }
}