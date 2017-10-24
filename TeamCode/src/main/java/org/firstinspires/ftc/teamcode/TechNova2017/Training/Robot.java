package org.firstinspires.ftc.teamcode.TechNova2017.Training;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {

    // NeveRest 40
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ; // NeveRest 40
    // This is < 1.0 if geared UP (48/84)
    static final double     DRIVE_GEAR_REDUCTION    = 48/84 *1.0;
    // For figuring out circumference
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;

    public static final double  COUNTS_PER_INCH         =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    HardwareMap hwMap             =  null;
    private DriveTrain driveTrain = null;

    /* default Constructor */
    public Robot(){
    }

    public Robot(HardwareMap hardwareMap){
        this.hwMap = hardwareMap;
    }

    public void init() {
        // create a new instance of DriveTrain
        driveTrain = new DriveTrain();

        driveTrain.init(hwMap);
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        driveTrain.setSpeed(leftSpeed, rightSpeed);
    }

    public boolean encoderDrive(double speed, double inches) {
        return driveTrain.setRunToPosition(speed, (int)(inches*COUNTS_PER_INCH));
    }

    public boolean fastTurn(double targetAngle) {
        return driveTrain.pidTurn(targetAngle, 0.5, 0.02);
    }

    public boolean mediumTurn(double targetAngle) {
        return driveTrain.pidTurn(targetAngle, 0.25, 0.02);
    }

    public boolean slowTurn(double targetAngle) {
        return driveTrain.pidTurn(targetAngle, 0.10, 0.025);
    }

    public void gyroDrive(double speed, double angle) {

    }

    public void stop() {
        driveTrain.stop();
    }

    public double getHeadingAngle() {
        return driveTrain.getHeadingAngle();
    }
}
