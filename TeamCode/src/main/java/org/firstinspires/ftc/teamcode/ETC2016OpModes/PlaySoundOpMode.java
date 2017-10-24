package org.firstinspires.ftc.teamcode.ETC2016OpModes;

import android.media.AudioManager;
import android.media.SoundPool;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.R;

/**
 * Created by terryw on 8/4/2016.
 */
@TeleOp(name = "Sound Test", group="Test")
@Disabled
public class PlaySoundOpMode extends OpMode {

    private SoundPool mySound;
    private int beepID;
    boolean loaded = false;

    public PlaySoundOpMode() {
    }

    @Override
    public void init() {
        mySound = new SoundPool(1, AudioManager.STREAM_MUSIC, 0); // PSM

//        beepID = mySound.load(hardwareMap.appContext, R.raw.siren, 1); // PSM
//
//        try {
//            Thread.sleep(200);
//        } catch (Exception e) {
//        }
//
//        mySound.play(beepID, 1, 1, 1, 0, 1);

        Log.i(PlaySoundOpMode.class.getSimpleName(), "Start ..");
//        mySound.setOnLoadCompleteListener(new SoundPool.OnLoadCompleteListener() {
//            public void onLoadComplete(SoundPool soundPool, int sampleId,int status) {
//                mySound.play(beepID, 1, 1, 1, 0, 1);
//                loaded = true;
//                Log.i(PlaySoundOpMode.class.getSimpleName(), "End ..");
//
//            }
//        });


//
//        beepID = mySound.load(hardwareMap.appContext, R.raw.beep1, 1); // PSM
//        try {
//            Thread.sleep(1000);
//        }catch(Exception e) {}
//
        mySound.play(beepID, 1, 1, 1, 0, 1);


    }

    @Override
    public void loop() {

    }
}
