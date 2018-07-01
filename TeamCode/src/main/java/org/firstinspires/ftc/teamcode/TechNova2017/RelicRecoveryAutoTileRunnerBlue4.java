package org.firstinspires.ftc.teamcode.TechNova2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Auto BLUE #1 Dump Glyphs", group = "Competition")
@Disabled
public class RelicRecoveryAutoTileRunnerBlue4 extends RelicRecoveryAutoTileRunnerBlue3 {

    @Override
    protected boolean dumpMoreGlyphs() {return true;
    }
}
