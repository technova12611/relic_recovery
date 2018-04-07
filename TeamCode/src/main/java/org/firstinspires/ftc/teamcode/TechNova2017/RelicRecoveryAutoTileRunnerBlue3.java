package org.firstinspires.ftc.teamcode.TechNova2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Auto BLUE #1 More Glyphs", group = "Competition")
@Disabled
public class RelicRecoveryAutoTileRunnerBlue3 extends RelicRecoveryAutoTileRunnerBlue1 {

    @Override
    protected boolean pickupMoreGlyphs() {
        return true;
    }
}
