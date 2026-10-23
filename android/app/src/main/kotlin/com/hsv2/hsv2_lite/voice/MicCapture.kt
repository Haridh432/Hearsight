package com.hsv2.hsv2_lite.voice

import android.annotation.SuppressLint
import android.media.AudioFormat
import android.media.AudioManager
import android.media.AudioRecord
import android.media.AudioRecordingConfiguration
import android.media.MediaRecorder
import android.media.audiofx.AcousticEchoCanceler
import android.media.audiofx.NoiseSuppressor
import android.util.Log

/**
 * Microphone capture at 16 kHz mono using VOICE_COMMUNICATION (echo-cancelled),
 * delivering float frames of [SileroVad.WINDOW] samples (512 samples = 32 ms) on a dedicated thread.
 */
class MicCapture(private val onFrame: (FloatArray) -> Unit) {

    @Volatile private var running = false
    private var thread: Thread? = null

    @SuppressLint("MissingPermission") // Callers check RECORD_AUDIO permission first
    fun start() {
        if (running) return
        val min = AudioRecord.getMinBufferSize(RATE, AudioFormat.CHANNEL_IN_MONO, AudioFormat.ENCODING_PCM_16BIT)
        val rec = AudioRecord(
            MediaRecorder.AudioSource.VOICE_COMMUNICATION, RATE, AudioFormat.CHANNEL_IN_MONO,
            AudioFormat.ENCODING_PCM_16BIT, maxOf(min, FRAME * 2 * 8),
        )
        check(rec.state == AudioRecord.STATE_INITIALIZED) { "Microphone initialization failed" }
        val aec = if (AcousticEchoCanceler.isAvailable()) {
            AcousticEchoCanceler.create(rec.audioSessionId)?.apply { enabled = true }
        } else null
        val ns = if (NoiseSuppressor.isAvailable()) {
            NoiseSuppressor.create(rec.audioSessionId)?.apply { enabled = true }
        } else null
        Log.i(TAG, "Mic: AEC=${aec != null}, NS=${ns != null}")

        var silenced = false
        rec.registerAudioRecordingCallback({ it.run() }, object : AudioManager.AudioRecordingCallback() {
            override fun onRecordingConfigChanged(configs: List<AudioRecordingConfiguration>) {
                val mine = configs.firstOrNull { it.clientAudioSessionId == rec.audioSessionId } ?: return
                if (mine.isClientSilenced != silenced) {
                    silenced = mine.isClientSilenced
                    Log.w(TAG, if (silenced) "Mic silenced by OS" else "Mic active again")
                }
            }
        })
        running = true
        thread = Thread({
            val shorts = ShortArray(FRAME)
            try {
                rec.startRecording()
                while (running) {
                    var n = 0
                    while (n < FRAME && running) {
                        val got = rec.read(shorts, n, FRAME - n)
                        if (got < 0) error("AudioRecord.read returned $got")
                        n += got
                    }
                    if (!running) break
                    onFrame(FloatArray(FRAME) { shorts[it] / 32768f })
                }
            } catch (t: Throwable) {
                Log.e(TAG, "Mic capture stopped", t)
            } finally {
                runCatching { rec.stop() }
                aec?.release()
                ns?.release()
                rec.release()
            }
        }, "hearsight-mic").apply {
            priority = Thread.MAX_PRIORITY
            start()
        }
    }

    fun stop() {
        running = false
        thread?.join(500)
        thread = null
    }

    companion object {
        const val RATE = 16000
        const val FRAME = SileroVad.WINDOW
        private const val TAG = "HSV2MicCapture"
    }
}
