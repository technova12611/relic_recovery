package org.firstinspires.ftc.teamcode.ETC2016OpModes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Training OpMode for FTC 9915 Summer Camp
 *
 * @author Terry Wang
 * @since 7/17/2016
 */
@TeleOp(name="Test: Cheesy Drive", group="Test")
@Disabled
public class TWCheesyTeleOp extends ETCBaseOpMode {

    private DcMotor chasisLeft;
    private DcMotor chasisRight;

    private CheesyDriveHelper cheesyDriveHelper;
    public TWCheesyTeleOp() {
    }

    @Override
    public void init() {
        // Right drive motor needs to run reverse.
        chasisLeft = hardwareMap.dcMotor.get("chassisLeft");

        chasisRight = hardwareMap.dcMotor.get("chassisRight");
        chasisRight.setDirection(DcMotor.Direction.REVERSE);

        cheesyDriveHelper = new CheesyDriveHelper(chasisLeft, chasisRight);

        telemetry.addData("00 OpMode: ", " --- TeleOps (Driver Control) ---");
    }

    @Override
    public void loop() {

        cheesyDrive();

        //--------------------------------------------------------
        //Send telemetry data to the driver station.
        updateGamepadTelemetry();
    }

    private void tankDrive() {
        // Map jobstick left to left motor and right joystick to right motor
        float leftDrivePower = scaleMotorPower(1 * gamepad1.left_stick_y);
        float rightDrivePower = scaleMotorPower(1 * gamepad1.right_stick_y);

        chasisLeft.setPower(leftDrivePower);
        chasisRight.setPower(rightDrivePower);
    }

    private void cheesyDrive() {
        // Map jobstick left to left motor and right joystick to right motor
        float wheel = scaleMotorPower(1 * gamepad1.left_stick_x);
        float throttle = scaleMotorPower(-1 * gamepad1.right_stick_y);

        cheesyDriveHelper.cheesyDrive(throttle,wheel,gamepad1.right_bumper, true);
    }

    @Override
    public AllianceColor getAllianceColor() {
        return AllianceColor.RED;
    }
}
