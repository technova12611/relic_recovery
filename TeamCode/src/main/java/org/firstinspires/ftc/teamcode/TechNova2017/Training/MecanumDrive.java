package org.firstinspires.ftc.teamcode.TechNova2017.Training;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.TechNova2017.BoschGyroSensor;
import org.firstinspires.ftc.teamcode.TechNova2017.util.DoubleToDoubleFunction;
import org.firstinspires.ftc.teamcode.TechNova2017.util.Values;

public class MecanumDrive {

    public static final double DEFAULT_MINIMUM_SPEED = 0.10;
    public static final double DEFAULT_MAXIMUM_SPEED = 1.0;
    public static final DoubleToDoubleFunction DEFAULT_SPEED_LIMITER = Values.symmetricLimiter(DEFAULT_MINIMUM_SPEED,
            DEFAULT_MAXIMUM_SPEED);

    private static final double SQRT_OF_TWO = Math.sqrt(2.0);
    private static final int NUMBER_OF_MOTORS = 4;
    private static final int LEFT_FRONT = 0;
    private static final int RIGHT_FRONT = 1;
    private static final int LEFT_REAR = 2;
    private static final int RIGHT_REAR = 3;
    private static final double OUTPUT_SCALE_FACTOR = 1.0;

    private final DcMotor leftFront;
    private final DcMotor leftRear;
    private final DcMotor rightFront;
    private final DcMotor rightRear;
    private final BoschGyroSensor gyro;
    private final DoubleToDoubleFunction speedLimiter;

    /**
     * Creates a new DriveSystem subsystem that uses the supplied drive train and no shifter. The voltage send to the drive
     * train is limited to [-1.0,1.0].
     *
     * @param leftFront the left front motor on the drive train for the robot; may not be null
     * @param leftRear the left rear motor on the drive train for the robot; may not be null
     * @param rightFront the right front motor on the drive train for the robot; may not be null
     * @param rightRear the right rear motor on the drive train for the robot; may not be null
     * @param gyro the gyroscope that will be used to determine the robot's direction for field-orientated controls; may not be
     *        null
     */
    public MecanumDrive(DcMotor leftFront, DcMotor leftRear, DcMotor rightFront, DcMotor rightRear, BoschGyroSensor gyro) {
        this(leftFront, leftRear, rightFront, rightRear, gyro, null);
    }

    /**
     * Creates a new DriveSystem subsystem that uses the supplied drive train and optional shifter. The voltage send to the
     * drive train is limited by the given function.
     *
     * @param leftFront the left front motor on the drive train for the robot; may not be null
     * @param leftRear the left rear motor on the drive train for the robot; may not be null
     * @param rightFront the right front motor on the drive train for the robot; may not be null
     * @param rightRear the right rear motor on the drive train for the robot; may not be null
     * @param gyro the gyroscope that will be used to determine the robot's direction for field-orientated controls; may not be
     *        null
     * @param speedLimiter the function that limits the speed sent to the drive train; if null, then a default clamping function
     *        is used to limit to the range [-1.0,1.0]
     */
    public MecanumDrive(DcMotor leftFront, DcMotor leftRear, DcMotor rightFront, DcMotor rightRear, BoschGyroSensor gyro,
                        DoubleToDoubleFunction speedLimiter) {
        this.leftFront = leftFront;
        this.leftRear = leftRear;
        this.rightFront = rightFront;
        this.rightRear = rightRear;
        this.gyro = gyro;
        this.speedLimiter = speedLimiter != null ? speedLimiter : DEFAULT_SPEED_LIMITER;
    }

    /**
     * Stop the drive train. This sets all motors to 0.
     */
    public void stop() {
        leftFront.setPower(0.0);
        rightFront.setPower(0.0);
        leftRear.setPower(0.0);
        rightRear.setPower(0.0);
    }

    /**
     * Cartesian drive method that specifies speeds in terms of the field longitudinal and lateral directions, using the drive's
     * angle sensor to automatically determine the robot's orientation relative to the field.
     * <p>
     * Using this method, the robot will move away from the drivers when the joystick is pushed forwards, and towards the
     * drivers when it is pulled towards them - regardless of what direction the robot is facing.
     *
     * @param x The speed that the robot should drive in the X direction. [-1.0..1.0]
     * @param y The speed that the robot should drive in the Y direction. This input is inverted to match the forward == -1.0
     *        that joysticks produce. [-1.0..1.0]
     * @param rotation The rate of rotation for the robot that is completely independent of the translation. [-1.0..1.0]
     */
    public void cartesian(double x, double y, double rotation) {
        double xIn = x;
        double yIn = y;
        // Negate y for the joystick.
        yIn = -yIn;
        // Compensate for gyro angle.
        double rotated[] = rotateVector(xIn, yIn, gyro.getRawHeadingAngle());
        xIn = rotated[0];
        yIn = rotated[1];

        double wheelSpeeds[] = new double[NUMBER_OF_MOTORS];
        wheelSpeeds[LEFT_FRONT] = xIn + yIn + rotation;
        wheelSpeeds[RIGHT_FRONT] = -xIn + yIn - rotation;
        wheelSpeeds[LEFT_REAR] = -xIn + yIn + rotation;
        wheelSpeeds[RIGHT_REAR] = xIn + yIn - rotation;

        normalize(wheelSpeeds);
        scale(wheelSpeeds, OUTPUT_SCALE_FACTOR);
        leftFront.setPower(wheelSpeeds[LEFT_FRONT]);
        leftRear.setPower(wheelSpeeds[LEFT_REAR]);
        rightFront.setPower(wheelSpeeds[RIGHT_FRONT]);
        rightRear.setPower(wheelSpeeds[RIGHT_REAR]);
    }

    /**
     * Polar drive method that specifies speeds in terms of magnitude and direction. This method does not use the drive's angle
     * sensor.
     *
     * @param magnitude The speed that the robot should drive in a given direction.
     * @param direction The direction the robot should drive in degrees. The direction and magnitude are independent of the
     *        rotation rate.
     * @param rotation The rate of rotation for the robot that is completely independent of the magnitude or direction.
     *        [-1.0..1.0]
     */
    public void polar(double magnitude, double direction, double rotation) {
        // Normalized for full power along the Cartesian axes.
        magnitude = speedLimiter.applyAsDouble(magnitude) * SQRT_OF_TWO;
        // The rollers are at 45 degree angles.
        double dirInRad = (direction + 45.0) * Math.PI / 180.0;
        double cosD = Math.cos(dirInRad);
        double sinD = Math.sin(dirInRad);

        double wheelSpeeds[] = new double[NUMBER_OF_MOTORS];
        wheelSpeeds[LEFT_FRONT] = (sinD * magnitude + rotation);
        wheelSpeeds[RIGHT_FRONT] = (cosD * magnitude - rotation);
        wheelSpeeds[LEFT_REAR] = (cosD * magnitude + rotation);
        wheelSpeeds[RIGHT_REAR] = (sinD * magnitude - rotation);

        normalize(wheelSpeeds);
        scale(wheelSpeeds, OUTPUT_SCALE_FACTOR);
        leftFront.setPower(wheelSpeeds[LEFT_FRONT]);
        leftRear.setPower(wheelSpeeds[LEFT_REAR]);
        rightFront.setPower(wheelSpeeds[RIGHT_FRONT]);
        rightRear.setPower(wheelSpeeds[RIGHT_REAR]);
    }

    /**
     * Normalize all wheel speeds if the magnitude of any wheel is greater than 1.0.
     * @param wheelSpeeds the speed of each motor
     */
    protected static void normalize(double wheelSpeeds[]) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        for (int i = 1; i < NUMBER_OF_MOTORS; i++) {
            double temp = Math.abs(wheelSpeeds[i]);
            if (maxMagnitude < temp) maxMagnitude = temp;
        }
        if (maxMagnitude > 1.0) {
            for (int i = 0; i < NUMBER_OF_MOTORS; i++) {
                wheelSpeeds[i] = wheelSpeeds[i] / maxMagnitude;
            }
        }
    }

    /**
     * Scale all speeds.
     * @param wheelSpeeds the speed of each motor
     * @param scaleFactor the scale factor to apply to the motor speeds
     */
    protected static void scale(double wheelSpeeds[], double scaleFactor) {
        for (int i = 1; i < NUMBER_OF_MOTORS; i++) {
            wheelSpeeds[i] = wheelSpeeds[i] * scaleFactor;
        }
    }

    /**
     * Rotate a vector in Cartesian space.
     * @param x the x value of the vector
     * @param y the y value of the vector
     * @param angle the angle to rotate
     * @return the vector of x and y values
     */
    protected static double[] rotateVector(double x, double y, double angle) {
        double angleInRadians = Math.toRadians(angle);
        double cosA = Math.cos(angleInRadians);
        double sinA = Math.sin(angleInRadians);
        double out[] = new double[2];
        out[0] = x * cosA - y * sinA;
        out[1] = x * sinA + y * cosA;
        return out;
    }
}