package org.firstinspires.ftc.teamcode.ETC2016OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by FTC 9915
 */
@Autonomous(name="Auto Red 1 One Particle Defense", group="9915 Red")
@Disabled
public class VelocityVortexAutoRED7 extends VelocityVortexAutoRED {

    public VelocityVortexAutoRED7() {
    }

    @Override
    protected int getNumOfParticles() {
        return 1;
    }

    @Override
    protected boolean isDefensivePlay() {return true;}
}

//**************************************************************************************************