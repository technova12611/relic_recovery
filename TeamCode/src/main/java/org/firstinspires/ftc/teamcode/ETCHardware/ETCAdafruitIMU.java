package org.firstinspires.ftc.teamcode.ETCHardware;

import android.util.Log;

import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cController;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.concurrent.locks.Lock;


/**
 * Created by Terry Wang, FTC team 9915
 * <p/>
 * This program is modified based on the library from Swerve Robotics.
 *
 * @since 10-30-2015
 */
public class ETCAdafruitIMU implements HardwareDevice, I2cController.I2cPortReadyCallback {

    public static final int BNO055_ADDRESS_A = 0x28;//From Adafruit_BNO055.h
    public static final int BNO055_ADDRESS_B = 0x29;
    public static final int BNO055_ID = 0xA0;
    public static final int


  /* For IMU mode, the register addresses 0X1A thru 0X2D (20 bytes) should be read consecutively */
  /* Euler data registers */
            BNO055_EULER_H_LSB_ADDR = 0X1A,
            BNO055_EULER_H_MSB_ADDR = 0X1B,
            BNO055_EULER_R_LSB_ADDR = 0X1C,
            BNO055_EULER_R_MSB_ADDR = 0X1D,
            BNO055_EULER_P_LSB_ADDR = 0X1E,
            BNO055_EULER_P_MSB_ADDR = 0X1F;

    public static final int                         //From Adafruit_BNO055.h
             /* Operation mode settings*/
            OPERATION_MODE_CONFIG                                   = 0X00,
            OPERATION_MODE_ACCONLY                                  = 0X01,
            OPERATION_MODE_MAGONLY                                  = 0X02,
            OPERATION_MODE_GYRONLY                                  = 0X03,
            OPERATION_MODE_ACCMAG                                   = 0X04,
            OPERATION_MODE_ACCGYRO                                  = 0X05,
            OPERATION_MODE_MAGGYRO                                  = 0X06,
            OPERATION_MODE_AMG                                      = 0X07,
            OPERATION_MODE_IMU                                      = 0X08, //Added to original C++ list
            OPERATION_MODE_IMUPLUS                                  = 0X08,
            OPERATION_MODE_COMPASS                                  = 0X09,
            OPERATION_MODE_M4G                                      = 0X0A,
            OPERATION_MODE_NDOF_FMC_OFF                             = 0X0B,
            OPERATION_MODE_NDOF                                     = 0X0C;

    private final int i2cBufferSize = 26; //Size of any extra buffers that will hold any incoming or
    // outgoing cache data
    private final I2cDevice i2cIMU; //The device class of the Adafruit/Bosch IMU
    private final int baseI2Caddress; //The base I2C address used to address all of the IMU's registers
    private int operationalMode;//The operational mode to which the IMU will be set after its initial
    private final byte[] i2cReadCache;//The interface will insert the bytes which have been read into
    private final byte[] i2cWriteCache; //This cache will hold the bytes which are to be written to
    private final Lock i2cReadCacheLock;//A lock on access to the IMU's I2C read cache
    private final Lock i2cWriteCacheLock; //A lock on access to the IMU's I2C write cache
    private boolean offsetsInitialized; //Flag indicating whether angle offsets have been
    double yawOffset = 0.0;
    double pitchOffset = 0.0;
    double rollOffset = 0.0;

    /* For IMU mode, the register addresses 0X1A thru 0X2D (20 bytes) should be read consecutively */
  /* Enable I2C Read Mode and address the bytes in the ReadCache using the following parameters: */
    private int numberOfRegisters = 20;

    private int registerStartAddress = BNO055_EULER_H_LSB_ADDR;
    private int readCacheOffset = BNO055_EULER_H_LSB_ADDR - I2cController.I2C_BUFFER_START_ADDRESS;

    /* The folowing variables instrument the performance of I2C reading and writing */
    public long totalI2Creads;//This variable counts the number of "read"s processed by the callback
    public double maxReadInterval;
    public double avgReadInterval;
    private long readStartTime;
    private long msExtra = 50;

    private void snooze(long milliSecs) {//Simple utility for sleeping (thereby releasing the CPU to
        // threads other than this one)
        try {
            Thread.sleep(milliSecs + msExtra);
        } catch (InterruptedException e) {
        }
    }
    /*
     * Operational modes are explained in the IMU datasheet in Table 3-3 on p.20, and elsewhere
     * in Section 3.3 which begins on p.20. A "fusion" mode must be chosen, in order for the IMU to
     * produce numerically integrated Euler angles from the gyros. Since FTC robotics activities
     * typically occur in environments that interfere with magnetic compasses, READ_MODE "fusion" mode like
     * "IMU" (READ_MODE.k.READ_MODE "IMUPLUS") is an appropriate choice, It runs only the accelerometers and the
     * gyros, and the Euler angles it generates for heading, pitch and yaw are relative to the
     * robot's initial orientation, not absolute angles relative to the inertial reference frame of
     * the earth's magnetic north and the earth's gravity.
    */

    public ETCAdafruitIMU(HardwareMap currentHWmap, String configuredIMUname,
                          byte baseAddress, byte operMode) throws RobotCoreException {

        i2cIMU = currentHWmap.i2cDevice.get(configuredIMUname);

        baseI2Caddress = (int) baseAddress & 0XFF;
        operationalMode = (int) operMode & 0XFF;
        i2cReadCache = i2cIMU.getI2cReadCache();
        i2cReadCacheLock = i2cIMU.getI2cReadCacheLock();
        i2cWriteCache = i2cIMU.getI2cWriteCache();
        i2cWriteCacheLock = i2cIMU.getI2cWriteCacheLock();

        initializeIMU();
    }

    public void startIMU() {
  /*
   * The IMU datasheet lists the following actions as necessary to initialize and set up the IMU,
   * asssuming that all operations of the Constructor completed successfully:
   * 1. Register the callback method which will keep the reading of IMU registers going
   * 2. Enable I2C read mode and start the self-perpetuating sequence of register readings
   *
  */
        i2cIMU.registerForI2cPortReadyCallback(this);
        offsetsInitialized = false;
        i2cIMU.enableI2cReadMode(I2cAddr.create8bit(baseI2Caddress), registerStartAddress, numberOfRegisters);
        i2cIMU.setI2cPortActionFlag();//Set this flag to do the next read
        i2cIMU.writeI2cCacheToController();

        maxReadInterval = 0.0;
        avgReadInterval = 0.0;
        totalI2Creads = 0L;
        readStartTime = System.nanoTime();
    }

    public double getHeadingAngle() {
        return getIMUGyroYawAngle();
    }
    /*
    * The IMU datasheet describes Euler angles in Section 3.6.5.4, and the quaternion in Section 3.6.5.5
    * both on p.35. Euler angles can be calculated from the 4 components of the quaternion, using the
    * Tait-Bryan equations listed in:
    * https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    */
    public double getIMUGyroYawAngle() {
        double tempYaw = 0.0;
        double yawAngle = 0.0;

        if (totalI2Creads > 2) { //Wait until the incoming data becomes valid
            try {
                i2cReadCacheLock.lock();
                /*
                * See IMU datasheet, Section 3.6.2 on p.30 AND Section 4.2.1 on pp. 51 & 52. IT APPEARS THAT
                * THE DOCUMENTATION HAS MISLABELED "ROLL" AS "PITCH" AND "PITCH" AS "ROLL". THIS FORCES
                * CORRECTIONS TO THE WAY THAT THE FIXED-POINT EULER DATA REGISTERS ARE USED: THE "EULER_R"
                * REGISTERS ARE TREATED AS PITCH DATA, AND THE "EULER_P" REGISTERS ARE TREATED AS ROLL DATA.
                */
                byte[] tempYByte = new byte[2];
                tempYByte[0] = i2cReadCache[BNO055_EULER_H_LSB_ADDR - readCacheOffset];
                tempYByte[1] = i2cReadCache[BNO055_EULER_H_MSB_ADDR - readCacheOffset];

                ByteBuffer bufferY = ByteBuffer.wrap(tempYByte).order(ByteOrder.LITTLE_ENDIAN);
                tempYaw = bufferY.getShort() / 16.0;

            } finally {
                i2cReadCacheLock.unlock();
            }

            //tempYaw = ((double) tempY) / 16.0;//Correct for the fixed-point scaling
            //Make tempYaw agree with tempQuatYaw with respect to sign and range of values: -180 to + 180 deg.
            tempYaw = -(tempYaw < 180.0 ? tempYaw : (tempYaw - 360.0));
            /*
            * The first angles read after IMU initialization are the effective "zeroes" for roll, pitch and
            * yaw angles. THEREFORE, IF THE IMU POWERS UP IN "IMU" MODE, THE PLATFORM ON WHICH IT IS MOUNTED
            * MUST BE MOUNTED ON A FLAT SURFACE (PITCH = 0 AND ROLL = 0) AND FACING "FORWARD" (YAW(HEADING)
            * = 0.
            */
            if (!offsetsInitialized) {
                yawOffset = tempYaw;
                offsetsInitialized = true;
                Log.i("FtcRobotController", String.format("Number of \"reads\" to initialize offsets: %d"
                        , totalI2Creads));
            }

            //Output yaw(heading) angles are offset-corrected and range-limited to -180 thru 180
            yawAngle = tempYaw - yawOffset;
            yawAngle = (yawAngle >= 180.0) ? (yawAngle - 360.0) : ((yawAngle < -180.0) ? (yawAngle + 360.0) : yawAngle);
        }

        return yawAngle;
    }


    public double getYawOffset() {
        double tempYaw = 0.0;
        double yawAngle = 0.0;
        try {
            i2cReadCacheLock.lock();
                /*
                * See IMU datasheet, Section 3.6.2 on p.30 AND Section 4.2.1 on pp. 51 & 52. IT APPEARS THAT
                * THE DOCUMENTATION HAS MISLABELED "ROLL" AS "PITCH" AND "PITCH" AS "ROLL". THIS FORCES
                * CORRECTIONS TO THE WAY THAT THE FIXED-POINT EULER DATA REGISTERS ARE USED: THE "EULER_R"
                * REGISTERS ARE TREATED AS PITCH DATA, AND THE "EULER_P" REGISTERS ARE TREATED AS ROLL DATA.
                */
            byte[] tempYByte = new byte[2];
            tempYByte[0] = i2cReadCache[BNO055_EULER_H_LSB_ADDR - readCacheOffset];
            tempYByte[1] = i2cReadCache[BNO055_EULER_H_MSB_ADDR - readCacheOffset];

            ByteBuffer bufferY = ByteBuffer.wrap(tempYByte).order(ByteOrder.LITTLE_ENDIAN);
            tempYaw = bufferY.getShort() / 16.0;

        } finally {
            i2cReadCacheLock.unlock();
        }

        //tempYaw = ((double) tempY) / 16.0;//Correct for the fixed-point scaling
        //Make tempYaw agree with tempQuatYaw with respect to sign and range of values: -180 to + 180 deg.
        tempYaw = -(tempYaw < 180.0 ? tempYaw : (tempYaw - 360.0));

        //Output yaw(heading) angles are offset-corrected and range-limited to -180 thru 180
        yawAngle = (tempYaw >= 180.0) ? (tempYaw - 360.0) : ((tempYaw < -180.0) ? (tempYaw + 360.0) : tempYaw);
        return yawAngle;
    }

//    private double[] getIMUGyroAngles() {
//        double tempYaw = 0.0;
//        double yawAngle = 0.0;
//
//        double tempPitch = 0.0;
//        double pitchAngle = 0.0;
//
//        double tempRoll = 0.0;
//        double rollAngle = 0.0;
//
//        if (totalI2Creads > 2) { //Wait until the incoming data becomes valid
//            try {
//                i2cReadCacheLock.lock();
//                /*
//                * See IMU datasheet, Section 3.6.2 on p.30 AND Section 4.2.1 on pp. 51 & 52. IT APPEARS THAT
//                * THE DOCUMENTATION HAS MISLABELED "ROLL" AS "PITCH" AND "PITCH" AS "ROLL". THIS FORCES
//                * CORRECTIONS TO THE WAY THAT THE FIXED-POINT EULER DATA REGISTERS ARE USED: THE "EULER_R"
//                * REGISTERS ARE TREATED AS PITCH DATA, AND THE "EULER_P" REGISTERS ARE TREATED AS ROLL DATA.
//                */
//                byte[] tempYByte = new byte[2];
//                tempYByte[0] = i2cReadCache[BNO055_EULER_H_LSB_ADDR - readCacheOffset];
//                tempYByte[1] = i2cReadCache[BNO055_EULER_H_MSB_ADDR - readCacheOffset];
//
//                ByteBuffer bufferY = ByteBuffer.wrap(tempYByte).order(ByteOrder.LITTLE_ENDIAN);
//                tempYaw = bufferY.getShort() / 16.0;
//
//                byte[] tempPByte = new byte[2];
//                tempPByte[0] = i2cReadCache[BNO055_EULER_P_LSB_ADDR - readCacheOffset];
//                tempPByte[1] = i2cReadCache[BNO055_EULER_P_MSB_ADDR - readCacheOffset];
//
//                ByteBuffer bufferP = ByteBuffer.wrap(tempPByte).order(ByteOrder.LITTLE_ENDIAN);
//                tempPitch = bufferP.getShort() / 16.0;
//
//                byte[] tempRByte = new byte[2];
//                tempRByte[0] = i2cReadCache[BNO055_EULER_R_LSB_ADDR - readCacheOffset];
//                tempRByte[1] = i2cReadCache[BNO055_EULER_R_MSB_ADDR - readCacheOffset];
//
//                ByteBuffer bufferR = ByteBuffer.wrap(tempRByte).order(ByteOrder.LITTLE_ENDIAN);
//                tempRoll = bufferR.getShort() / 16.0;
//            } finally {
//                i2cReadCacheLock.unlock();
//            }
//
//            //tempYaw = ((double) tempY) / 16.0;//Correct for the fixed-point scaling
//            //Make tempYaw agree with tempQuatYaw with respect to sign and range of values: -180 to + 180 deg.
//            tempYaw = -(tempYaw < 180.0 ? tempYaw : (tempYaw - 360.0));
//            tempPitch = -(tempPitch < 180.0 ? tempPitch : (tempPitch - 360.0));
//            tempRoll = -(tempRoll < 180.0 ? tempRoll : (tempRoll - 360.0));
//            /*
//            * The first angles read after IMU initialization are the effective "zeroes" for roll, pitch and
//            * yaw angles. THEREFORE, IF THE IMU POWERS UP IN "IMU" MODE, THE PLATFORM ON WHICH IT IS MOUNTED
//            * MUST BE MOUNTED ON A FLAT SURFACE (PITCH = 0 AND ROLL = 0) AND FACING "FORWARD" (YAW(HEADING)
//            * = 0.
//            */
//            if (!offsetsInitialized) {
//                yawOffset = tempYaw;
//                pitchOffset = tempPitch;
//                rollOffset = tempRoll;
//                offsetsInitialized = true;
//                Log.i("FtcRobotController", String.format("Number of \"reads\" to initialize offsets: %d"
//                        , totalI2Creads));
//            }
//
//            //Output yaw(heading) angles are offset-corrected and range-limited to -180 thru 180
//            yawAngle = tempYaw - yawOffset;
//            yawAngle = (yawAngle >= 180.0) ? (yawAngle - 360.0) : ((yawAngle < -180.0) ? (yawAngle + 360.0) : yawAngle);
//
//            pitchAngle = tempPitch - pitchOffset;
//            pitchAngle = (pitchAngle >= 180.0) ? (pitchAngle - 360.0) : ((pitchAngle < -180.0) ? (pitchAngle + 360.0) : pitchAngle);
//
//            rollAngle = tempRoll - rollOffset;
//            rollAngle = (rollAngle >= 180.0) ? (rollAngle - 360.0) : ((rollAngle < -180.0) ? (rollAngle + 360.0) : rollAngle);
//        }
//
//        double[] angles = {yawAngle, pitchAngle, rollAngle};
//
//        return angles;
//    }

    /*
     * Use of the following callback assumes that I2C reading has been enabled for READ_MODE particular I2C
     * register address (as the starting address) and READ_MODE particular byte count. Registration of this
     * callback should only take place when that reading enablement is done.
    */
    public void portIsReady(int port) { //Implementation of I2cController.I2cPortReadyCallback
        double latestInterval;
        if ((latestInterval = ((System.nanoTime() - readStartTime) / 1000000.0)) > maxReadInterval)
            maxReadInterval = latestInterval;
        avgReadInterval = ((avgReadInterval * 511.0) + latestInterval) / 512.0;
        i2cIMU.readI2cCacheFromController(); //Read in the most recent data from the device
        totalI2Creads++;
        i2cIMU.setI2cPortActionFlag();   //Set this flag to do the next read
        i2cIMU.writeI2cPortFlagOnlyToController();
        readStartTime = System.nanoTime();
        totalI2Creads++;
        //At this point, the port becomes busy (not ready) doing the next read
    }

    //All of the following implement the HardwareDevice Interface
    public String getDeviceName() {
        return "Bosch 9-DOF IMU BNO055";
    }

    public String getConnectionInfo() {
        return i2cIMU.getConnectionInfo();
    }

    public int getVersion() {
        return BNO055_ID;
    } //Temporarily

    public void close() {
    }

    final static byte bCHIP_ID_VALUE = (byte) 0xa0;
    private static final int msAwaitChipId = 5000;
    private static final int msAwaitSelfTest = 3000;

    private void initializeIMU() {

        ElapsedTime elapsed = new ElapsedTime();

        // Lore: "send a throw-away command [...] just to make sure the BNO is in a good state
        // and ready to accept commands (this seems to be necessary after a hard power down)."
        byte[] outboundBytes = new byte[1];
        this.write8(REGISTER.PAGE_ID, (byte) 0x00);


        // Make sure we have the right device
        byte chipId = read8(REGISTER.CHIP_ID);
        Log.i("ETCAdafruitIMU","Reading Chip ID: " + chipId);

        if (chipId != bCHIP_ID_VALUE) {
            snooze(650);     // delay value is from from Table 0-2
            chipId = read8(REGISTER.CHIP_ID);
            if (chipId != bCHIP_ID_VALUE)
                throw new RuntimeException("I2C Adafruit IMU invalid chipId:" + chipId);
        }

        Log.i("ETCAdafruitIMU","Set sensor mode to Config");
        // Make sure we are in config mode
        setSensorMode(SENSOR_MODE.CONFIG);

        // Reset the system, and wait for the chip id register to switch back from its reset state
        // to the it's chip id state. This can take a very long time, some 650ms (Table 0-2, p13)
        // perhaps. While in the reset state the chip id (and other registers) reads as 0xFF.
        elapsed.reset();

        Log.i("ETCAdafruitIMU", "Reset system, may take a long time.");
        write8(REGISTER.SYS_TRIGGER, (byte) 0x20);
        for (; ; ) {
            chipId = read8(REGISTER.CHIP_ID);
            if (chipId == bCHIP_ID_VALUE)
                break;
            snooze(10);
            if (milliseconds(elapsed) > msAwaitChipId)
                throw new RuntimeException("Adafruit init failed to retrieve chip id");
        }
        snooze(50);

        Log.i("ETCAdafruitIMU", "Set power mode to Normal");
        // Set to normal power mode
        write8(REGISTER.PWR_MODE, POWER_MODE.NORMAL.getValue());
        snooze(10);

        // Make sure we're looking at register page zero, as the other registers
        // we need to set here are on that page.
        Log.i("ETCAdafruitIMU","Set Page ID to 0");
        write8(REGISTER.PAGE_ID, (byte) 0x00);

        Parameters parameters = new Parameters();

        // Set the output units. Section 3.6, p31
        int unitsel = (parameters.pitchmode.bVal << 7) |       // pitch angle convention
                (parameters.temperatureUnit.bVal << 4) | // temperature
                (parameters.angleunit.bVal << 2) |       // euler angle units
                (parameters.angleunit.bVal << 1) |       // gyro units, per second
                (parameters.accelunit.bVal /*<< 0*/);    // accelerometer units

        Log.i("ETCAdafruitIMU","Set default unit selection");
        write8(REGISTER.UNIT_SEL, (byte) unitsel);

        // Use or don't use the external crystal
        // See Section 5.5 (p100) of the BNO055 specification.
        Log.i("ETCAdafruitIMU","Set using external crystal");
        write8(REGISTER.SYS_TRIGGER, (byte) (parameters.useExternalCrystal ? 0x80 : 0x00));
        snooze(10);

        // Run a self test. This appears to be a necessary step in order for the
        // sensor to be able to actually be used.

        Log.i("ETCAdafruitIMU","Set Self Test mode");
        write8(REGISTER.SYS_TRIGGER, (byte) (read8(REGISTER.SYS_TRIGGER) | 0x01));           // SYS_TRIGGER=0x3F
        elapsed.reset();

        boolean selfTestSuccessful = false;
        while (!selfTestSuccessful && milliseconds(elapsed) < msAwaitSelfTest) {
            selfTestSuccessful = (read8(REGISTER.SELFTEST_RESULT) & 0x0F) == 0x0F;    // SELFTEST_RESULT=0x36
        }

        if (!selfTestSuccessful)
            throw new RuntimeException("Ada fruit IMU self test failed");

        // Finally, enter the requested operating mode (see section 3.3)
        Log.i("ETCAdafruitIMU","Set Sensor to IMU Mode");
        setSensorMode(SENSOR_MODE.IMU);
        snooze(200);

        Log.i("ETCAdafruitIMU", "IMU initialized successfully ...");
    }


    /* The default operation mode after power-on is CONFIGMODE. When the user changes to another
    operation mode, the sensors which are required in that particular sensor mode are powered,
    while the sensors whose signals are not required are set to suspend mode. */
    private void setSensorMode(SENSOR_MODE mode) {
        // Actually change the operation/sensor mode
        this.write8(REGISTER.OPR_MODE, (byte) (mode.bVal & 0x0F));          // OPR_MODE=0x3D

        // Delay per Table 3-6 of BNO055 Data sheet (p21)
        if (mode == SENSOR_MODE.CONFIG)
            snooze(19);
        else
            snooze(7);
    }

    static public double milliseconds(ElapsedTime elapsed) {
        return elapsed.time() * 1000.0;
    }

    private boolean write8(REGISTER reg, byte data) {
        long rightNow = System.nanoTime(), startTime = System.nanoTime(), timeOut = 3*1000000000L;

        try {
            while ((!i2cIMU.isI2cPortReady())
                    && (((rightNow = System.nanoTime()) - startTime) < timeOut)) {
                Thread.sleep(250);//"Snooze" right here, until the port is ready (READ_MODE good thing) OR n billion
                //nanoseconds pass with the port "stuck busy" (READ_MODE VERY bad thing)
            }
        } catch (InterruptedException e) {
            Log.i("ETCAdafruitIMU", "Unexpected interrupt while \"sleeping\" in i2cWriteImmediately.");
            return false;
        }

        try {
            i2cWriteCacheLock.lock();
            i2cWriteCache[I2cController.I2C_BUFFER_START_ADDRESS] = data;
        } finally {
            i2cWriteCacheLock.unlock();
        }

        i2cIMU.enableI2cWriteMode(I2cAddr.create8bit(baseI2Caddress), reg.bVal, 1);
        i2cIMU.setI2cPortActionFlag();  //Set the "go do it" flag
        i2cIMU.writeI2cCacheToController(); //Then write it and the cache out
        snooze(20);//Give the data time to go from the Interface Module to the IMU hardware
        return true;
    }

    private byte read8(REGISTER reg) {
        boolean readingEnabled = false;
        long readStart = System.nanoTime(), rightNow = System.nanoTime(),
                loopStart = System.nanoTime(), timeout = 3 * 1000000000L;

        byte result = 0X00;

        while ((System.nanoTime() - readStart) <= 10 * 1000000000L) {//Set READ_MODE 1-minute overall timeout
            try {
                loopStart = System.nanoTime();
                while ((!i2cIMU.isI2cPortReady())
                        && (((rightNow = System.nanoTime()) - loopStart) < timeout)) {
                    Thread.sleep(250);//"Snooze" right here, until the port is ready (READ_MODE good thing) OR n billion
                    //nanoseconds pass with the port "stuck busy" (READ_MODE VERY bad thing)
                }
            } catch (InterruptedException e) {
                Log.i("ETCAdafruitIMU", "Unexpected interrupt while \"sleeping\" in autoCalibrationOK.");
                return result;
            }

            if (!readingEnabled) {//Start READ_MODE stream of reads of the calibration status byte
                i2cIMU.enableI2cReadMode(I2cAddr.create8bit(baseI2Caddress), reg.bVal, 1);
                i2cIMU.setI2cPortActionFlag();//Set this flag to do the next read
                i2cIMU.writeI2cCacheToController();
                snooze(100);//Give the data time to go from the Interface Module to the IMU hardware
                readingEnabled = true;
            } else {//Check the Calibration Status and Self-Test bytes in the Read Cache. IMU datasheet
                // Sec. 3.10, p.47. Also, see p. 70
                i2cIMU.readI2cCacheFromController();//Read in the most recent data from the device
                snooze(100);//Give the data time to come into the Interface Module from the IMU hardware
                try {
                    i2cReadCacheLock.lock();
                    return i2cReadCache[I2cController.I2C_BUFFER_START_ADDRESS];
                } finally {
                    i2cReadCacheLock.unlock();
                }
            }

            //At this point, the port becomes busy (not ready) doing the next read
            snooze(50);//Give the data time to go from the Interface Module to the IMU hardware
        }

        return result;
    }

    class Parameters {
        /**
         * the address at which the sensor resides on the I2C bus.
         */
        public I2CADDR i2cAddr8Bit = I2CADDR.DEFAULT;

        /**
         * the mode we wish to use the sensor in
         */
        public SENSOR_MODE mode = SENSOR_MODE.IMU;

        /**
         * whether to use the external or internal 32.768khz crystal. External crystal
         * use is recommended by the BNO055 specification.
         */
        public boolean useExternalCrystal = true;

        /**
         * units in which temperature are measured. See Section 3.6.1 (p31) of the BNO055 specification
         */
        public TEMPUNIT temperatureUnit = TEMPUNIT.CELSIUS;
        /**
         * units in which angles and angular rates are measured. See Section 3.6.1 (p31) of the BNO055 specification
         */
        public ANGLEUNIT angleunit = ANGLEUNIT.DEGREES;
        /**
         * units in which accelerations are measured. See Section 3.6.1 (p31) of the BNO055 specification
         */
        public ACCELUNIT accelunit = ACCELUNIT.METERS_PERSEC_PERSEC;
        /**
         * directional convention for measuring pitch angles. See Section 3.6.1 (p31) of the BNO055 specification
         */
        public PITCHMODE pitchmode = PITCHMODE.ANDROID;    // Section 3.6.2
    }

    /**
     * REGISTER provides symbolic names for each of the BNO055 device registers
     */
    enum REGISTER {
        /**
         * Controls which of the two register pages are visible
         */
        PAGE_ID(0X07),

        CHIP_ID(0x00),
        ACCEL_REV_ID(0x01),
        MAG_REV_ID(0x02),
        GYRO_REV_ID(0x03),
        SW_REV_ID_LSB(0x04),
        SW_REV_ID_MSB(0x05),
        BL_REV_ID(0X06),

        /**
         * Acceleration data register
         */
        ACCEL_DATA_X_LSB(0X08),
        ACCEL_DATA_X_MSB(0X09),
        ACCEL_DATA_Y_LSB(0X0A),
        ACCEL_DATA_Y_MSB(0X0B),
        ACCEL_DATA_Z_LSB(0X0C),
        ACCEL_DATA_Z_MSB(0X0D),

        /**
         * Magnetometer data register
         */
        MAG_DATA_X_LSB(0X0E),
        MAG_DATA_X_MSB(0X0F),
        MAG_DATA_Y_LSB(0X10),
        MAG_DATA_Y_MSB(0X11),
        MAG_DATA_Z_LSB(0X12),
        MAG_DATA_Z_MSB(0X13),

        /**
         * Gyro data registers
         */
        GYRO_DATA_X_LSB(0X14),
        GYRO_DATA_X_MSB(0X15),
        GYRO_DATA_Y_LSB(0X16),
        GYRO_DATA_Y_MSB(0X17),
        GYRO_DATA_Z_LSB(0X18),
        GYRO_DATA_Z_MSB(0X19),

        /**
         * Euler data registers
         */
        EULER_H_LSB(0X1A),
        EULER_H_MSB(0X1B),
        EULER_R_LSB(0X1C),
        EULER_R_MSB(0X1D),
        EULER_P_LSB(0X1E),
        EULER_P_MSB(0X1F),

        /**
         * Quaternion data registers
         */
        QUATERNION_DATA_W_LSB(0X20),
        QUATERNION_DATA_W_MSB(0X21),
        QUATERNION_DATA_X_LSB(0X22),
        QUATERNION_DATA_X_MSB(0X23),
        QUATERNION_DATA_Y_LSB(0X24),
        QUATERNION_DATA_Y_MSB(0X25),
        QUATERNION_DATA_Z_LSB(0X26),
        QUATERNION_DATA_Z_MSB(0X27),

        /**
         * Linear acceleration data registers
         */
        LINEAR_ACCEL_DATA_X_LSB(0X28),
        LINEAR_ACCEL_DATA_X_MSB(0X29),
        LINEAR_ACCEL_DATA_Y_LSB(0X2A),
        LINEAR_ACCEL_DATA_Y_MSB(0X2B),
        LINEAR_ACCEL_DATA_Z_LSB(0X2C),
        LINEAR_ACCEL_DATA_Z_MSB(0X2D),

        /**
         * Gravity data registers
         */
        GRAVITY_DATA_X_LSB(0X2E),
        GRAVITY_DATA_X_MSB(0X2F),
        GRAVITY_DATA_Y_LSB(0X30),
        GRAVITY_DATA_Y_MSB(0X31),
        GRAVITY_DATA_Z_LSB(0X32),
        GRAVITY_DATA_Z_MSB(0X33),

        /**
         * Temperature data register
         */
        TEMP(0X34),

        /**
         * Status registers
         */
        CALIB_STAT(0X35),
        SELFTEST_RESULT(0X36),
        INTR_STAT(0X37),

        SYS_CLK_STAT(0X38),
        SYS_STAT(0X39),
        SYS_ERR(0X3A),

        /**
         * Unit selection register
         */
        UNIT_SEL(0X3B),
        DATA_SELECT(0X3C),

        /**
         * Mode registers
         */
        OPR_MODE(0X3D),
        PWR_MODE(0X3E),

        SYS_TRIGGER(0X3F),
        TEMP_SOURCE(0X40),

        /**
         * Axis remap registers
         */
        AXIS_MAP_CONFIG(0X41),
        AXIS_MAP_SIGN(0X42),

        /**
         * SIC registers
         */
        SIC_MATRIX_0_LSB(0X43),
        SIC_MATRIX_0_MSB(0X44),
        SIC_MATRIX_1_LSB(0X45),
        SIC_MATRIX_1_MSB(0X46),
        SIC_MATRIX_2_LSB(0X47),
        SIC_MATRIX_2_MSB(0X48),
        SIC_MATRIX_3_LSB(0X49),
        SIC_MATRIX_3_MSB(0X4A),
        SIC_MATRIX_4_LSB(0X4B),
        SIC_MATRIX_4_MSB(0X4C),
        SIC_MATRIX_5_LSB(0X4D),
        SIC_MATRIX_5_MSB(0X4E),
        SIC_MATRIX_6_LSB(0X4F),
        SIC_MATRIX_6_MSB(0X50),
        SIC_MATRIX_7_LSB(0X51),
        SIC_MATRIX_7_MSB(0X52),
        SIC_MATRIX_8_LSB(0X53),
        SIC_MATRIX_8_MSB(0X54),

        /**
         * Accelerometer Offset registers
         */
        ACCEL_OFFSET_X_LSB(0X55),
        ACCEL_OFFSET_X_MSB(0X56),
        ACCEL_OFFSET_Y_LSB(0X57),
        ACCEL_OFFSET_Y_MSB(0X58),
        ACCEL_OFFSET_Z_LSB(0X59),
        ACCEL_OFFSET_Z_MSB(0X5A),

        /**
         * Magnetometer Offset registers
         */
        MAG_OFFSET_X_LSB(0X5B),
        MAG_OFFSET_X_MSB(0X5C),
        MAG_OFFSET_Y_LSB(0X5D),
        MAG_OFFSET_Y_MSB(0X5E),
        MAG_OFFSET_Z_LSB(0X5F),
        MAG_OFFSET_Z_MSB(0X60),

        /**
         * Gyroscope Offset register s
         */
        GYRO_OFFSET_X_LSB(0X61),
        GYRO_OFFSET_X_MSB(0X62),
        GYRO_OFFSET_Y_LSB(0X63),
        GYRO_OFFSET_Y_MSB(0X64),
        GYRO_OFFSET_Z_LSB(0X65),
        GYRO_OFFSET_Z_MSB(0X66),

        /**
         * Radius registers
         */
        ACCEL_RADIUS_LSB(0X67),
        ACCEL_RADIUS_MSB(0X68),
        MAG_RADIUS_LSB(0X69),
        MAG_RADIUS_MSB(0X6A);
        //------------------------------------------------------------------------------------------
        public final byte bVal;

        private REGISTER(int i) {
            this.bVal = (byte) i;
        }
    }

    enum SENSOR_MODE {
        CONFIG(0X00), ACCONLY(0X01), MAGONLY(0X02),
        GYRONLY(0X03), ACCMAG(0X04), ACCGYRO(0X05),
        MAGGYRO(0X06), AMG(0X07), IMU(0X08),
        COMPASS(0X09), M4G(0X0A), NDOF_FMC_OFF(0X0B),
        NDOF(0X0C);
        //------------------------------------------------------------------------------------------
        public final byte bVal;

        SENSOR_MODE(int i) {
            this.bVal = (byte) i;
        }
    }

    enum POWER_MODE {
        NORMAL(0X00),
        LOWPOWER(0X01),
        SUSPEND(0X02);
        //------------------------------------------------------------------------------------------
        private byte value;

        private POWER_MODE(int value) {
            this.value = (byte) value;
        }

        public byte getValue() {
            return this.value;
        }
    }

    //----------------------------------------------------------------------------------------------
    // Enumerations to make all of the above work
    //----------------------------------------------------------------------------------------------

    enum I2CADDR {
        UNSPECIFIED(-1), DEFAULT(0x28 * 2), ALTERNATE(0x29 * 2);
        public final byte bVal;

        I2CADDR(int i) {
            bVal = (byte) i;
        }
    }

    enum TEMPUNIT {
        CELSIUS(0), FARENHEIT(1);
        public final byte bVal;

        TEMPUNIT(int i) {
            bVal = (byte) i;
        }
    }

    enum ANGLEUNIT {
        DEGREES(0), RADIANS(1);
        public final byte bVal;

        ANGLEUNIT(int i) {
            bVal = (byte) i;
        }
    }

    enum ACCELUNIT {
        METERS_PERSEC_PERSEC(0), MILLIGALS(1);
        public final byte bVal;

        ACCELUNIT(int i) {
            bVal = (byte) i;
        }
    }

    enum PITCHMODE {
        WINDOWS(0), ANDROID(1);
        public final byte bVal;

        PITCHMODE(int i) {
            bVal = (byte) i;
        }
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Adafruit;
    }
}
