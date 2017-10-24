package org.firstinspires.ftc.teamcode.ETC2016OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by FTC 9915
 */
@Autonomous(name="Auto Red 1 Defense", group="9915 Red")
@Disabled
public class VelocityVortexAutoRED2 extends VelocityVortexAutoRED {

    public VelocityVortexAutoRED2() {
    }

    @Override
    protected boolean isDefensivePlay() {
        return true;
    }
}

//**************************************************************************************************