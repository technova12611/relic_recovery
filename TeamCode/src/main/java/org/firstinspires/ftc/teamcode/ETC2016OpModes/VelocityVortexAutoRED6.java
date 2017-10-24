package org.firstinspires.ftc.teamcode.ETC2016OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by FTC 9915
 */
@Autonomous(name="Auto Red 1 One Particle", group="9915 Red")
@Disabled
public class VelocityVortexAutoRED6 extends VelocityVortexAutoRED {

    public VelocityVortexAutoRED6() {
    }

    @Override
    protected int getNumOfParticles() {
        return 1;
    }
}

//**************************************************************************************************