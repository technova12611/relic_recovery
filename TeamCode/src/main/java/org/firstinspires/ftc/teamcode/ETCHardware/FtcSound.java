package org.firstinspires.ftc.teamcode.ETCHardware;

import android.media.AudioFormat;
import android.media.AudioManager;
import android.media.AudioTrack;

/**
 * This class implements a platform dependent sound object that makes playing a sound easy.
 * You can play a sound by providing a sound data buffer or you can play a sound with a
 * specified frequency and duration.
 */
public class FtcSound implements AudioTrack.OnPlaybackPositionUpdateListener
{
    private static final String moduleName = "FtcSound";
    private static final boolean debugEnabled = false;

    private static final FtcSound.WaveForm DEF_WAVEFORM = FtcSound.WaveForm.SawToothWave;
    private static final int DEF_SAMPLERATE = 16*1024;  // 16kHz

    /**
     * This enum type specifies the sound wave form to be used in playing a note.
     */
    public enum WaveForm
    {
        SineWave,
        SquareWave,
        SawToothWave
    }   //enum WaveForm

    private WaveForm waveForm;
    private int sampleRate;
    private AudioTrack audioTrack = null;
    private boolean playing = false;

    /**
     * Constructor: Create and initialize an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param waveForm specifies the sound wave form of the sound.
     * @param sampleRate specifies the sampling rate.
     */
    public FtcSound(String instanceName, WaveForm waveForm, int sampleRate)
    {
        this.waveForm = waveForm;
        this.sampleRate = sampleRate;
    }   //FtcSound

    /**
     * Constructor: Create and initialize an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public FtcSound(String instanceName)
    {
        this(instanceName, DEF_WAVEFORM, DEF_SAMPLERATE);
    }   //FtcSound

    /**
     * This method plays a tone with the specified frequency and duration.
     *
     * @param frequency specifies the tone frequency in Hz.
     * @param duration specifies the duration in seconds.
     */
    public void playTone(double frequency, double duration)
    {
        switch (waveForm)
        {
            case SineWave:
                playSineWave(frequency, duration);
                break;

            case SquareWave:
                playSquareWave(frequency, duration);
                break;

            case SawToothWave:
                playSawToothWave(frequency, duration);
                break;
        }
    }   //playTone

    /**
     * This method plays a sine wave tone with the specified frequency and duration.
     *
     * @param frequency specifies the tone frequency in Hz.
     * @param duration specifies the duration in seconds.
     */
    public void playSineWave(double frequency, double duration)
    {
        playSound(genSineWave(frequency, duration));
    }   //playSineWave

    /**
     * This method plays a square wave tone with the specified frequency and duration.
     *
     * @param frequency specifies the tone frequency in Hz.
     * @param duration specifies the duration in seconds.
     */
    public void playSquareWave(double frequency, double duration)
    {
        playSound(genSquareWave(frequency, duration));
    }   //playSquareWave

    /**
     * This method plays a sawtooth wave tone with the specified frequency and duration.
     *
     * @param frequency specifies the tone frequency in Hz.
     * @param duration specifies the duration in seconds.
     */
    public void playSawToothWave(double frequency, double duration)
    {
        playSound(genSawToothWave(frequency, duration));
    }   //playSawToothWave

    /**
     * This method plays the sound waveform specified in the given data array.
     * It also sets a completion notification handler.
     *
     * @param data Specifies the data array.
     */
    public void playSound(short[] data)
    {
        audioTrack = new AudioTrack(
                AudioManager.STREAM_MUSIC,
                sampleRate,
                AudioFormat.CHANNEL_OUT_MONO,
                AudioFormat.ENCODING_PCM_16BIT,
                data.length*2,                      //data length in bytes
                AudioTrack.MODE_STATIC);
        audioTrack.write(data, 0, data.length);
        audioTrack.setNotificationMarkerPosition(data.length);
        audioTrack.setPlaybackPositionUpdateListener(this);
        audioTrack.play();
        playing = true;
    }   //playSound

    /**
     * This method stops the playing of the sound in progress.
     */
    public void stop()
    {
        if (playing)
        {
            audioTrack.stop();
            playing = false;
        }
    }   //stop

    /**
     * This method checks if the sound is still playing.
     *
     * @return true if the sound is still playing, false otherwise.
     */
    public boolean isPlaying()
    {
        return playing;
    }   //isPlaying

    /**
     * This method generates the wave data for a sine wave with the specified frequency and duration.
     *
     * @param frequency specifies the tone frequency in Hz.
     * @param duration specifies the duration in seconds.
     * @return generated wave data array.
     */
    private short[] genSineWave(double frequency, double duration)
    {
        int numSamples = (int)(sampleRate*duration);
        short[] buffer = new short[numSamples];

        for (int i = 0; i < numSamples; i++)
        {
            buffer[i] = (short)(Math.sin(2*Math.PI*i/(sampleRate/frequency))*Short.MAX_VALUE);
        }

        return buffer;
    }   //genSineWave

    /**
     * This method generates the wave data for a square wave with the specified frequency and duration.
     *
     * @param frequency specifies the tone frequency in Hz.
     * @param duration specifies the duration in seconds.
     * @return generated wave data array.
     */
    private short[] genSquareWave(double frequency, double duration)
    {
        int numSamples = (int)(sampleRate*duration);
        short[] buffer = new short[numSamples];

        for (int i = 0; i < numSamples; i++)
        {
            buffer[i] = ((i/(sampleRate/frequency))%1.0 >= 0.5)? Short.MIN_VALUE: Short.MAX_VALUE;
        }

        return buffer;
    }   //genSquareWave

    /**
     * This method generates the wave data for a sawtooth wave with the specified frequency and duration.
     *
     * @param frequency specifies the tone frequency in Hz.
     * @param duration specifies the duration in seconds.
     * @return generated wave data array.
     */
    private short[] genSawToothWave(double frequency, double duration)
    {
        int numSamples = (int)(sampleRate*duration);
        short[] buffer = new short[numSamples];

        for (int i = 0; i < numSamples; i++)
        {
            double phase = (i/(sampleRate/frequency))%1.0;
            buffer[i] = (short)(
                    phase < 0.25?
                            4.0*phase*Short.MAX_VALUE:
                            phase < 0.75?
                                    (1 - (phase - 0.25)*2)*(Short.MAX_VALUE - Short.MIN_VALUE) + Short.MIN_VALUE:
                                    (1 - (phase - 0.75)*4)*Short.MIN_VALUE
            );
        }

        return buffer;
    }   //genSawToothWave

    //
    // Implements AudioTrack.OnPlaybackPositionUpdateListener interfafce.
    //

    /**
     * This method is called when the sample at the set marker has been played. This is used to indicate
     * the completion of the tone played.
     *
     * @param track specifies the AudioTrack object that was playing.
     */
    @Override
    public void onMarkerReached(AudioTrack track)
    {
        audioTrack.setNotificationMarkerPosition(0);
        playing = false;
    }   //onMarkerReached

    /**
     * This method is called when the period marker has been reached.
     *
     * @param track specifies the AudioTrack object that was playing.
     */
    @Override
    public void onPeriodicNotification(AudioTrack track)
    {
    }   //onPeriodicNotification

}   //class FtcSound
