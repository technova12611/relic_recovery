package org.firstinspires.ftc.teamcode.ETCHardware;

import com.qualcomm.robotcore.hardware.I2cController;
import com.qualcomm.robotcore.hardware.I2cControllerPortDevice;

public class FtcI2cDeviceState
{
    I2cController i2cController;
    int port;
    private I2cController.I2cPortReadyCallback deviceCallback;
    private boolean deviceEnabled;

    public FtcI2cDeviceState(I2cControllerPortDevice device)
    {
        i2cController = device.getI2cController();
        port = device.getPort();
        deviceCallback = i2cController.getI2cPortReadyCallback(port);
        deviceEnabled = true;
    }   //FtcI2cDeviceState

    public boolean isEnabled()
    {
        return deviceEnabled;
    }   //isEnabled

    public void setEnabled(boolean enabled)
    {
        if (deviceEnabled != enabled)
        {
            if (enabled)
            {
                i2cController.registerForI2cPortReadyCallback(deviceCallback, port);
            }
            else
            {
                i2cController.deregisterForPortReadyCallback(port);
            }
            deviceEnabled = enabled;
        }
    }   //setEnabled

}   //class FtcI2cDeviceState
