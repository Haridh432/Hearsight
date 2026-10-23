package com.hsv2.hsv2_lite.voice

import android.content.Context
import android.media.AudioManager
import android.media.ToneGenerator
import android.os.VibrationEffect
import android.os.VibratorManager

/**
 * Haptic and tone acknowledgement when the assistant begins listening or processes a command.
 */
object Earcon {
    fun wake(context: Context) {
        runCatching {
            context.getSystemService(VibratorManager::class.java)?.defaultVibrator
                ?.vibrate(VibrationEffect.createPredefined(VibrationEffect.EFFECT_HEAVY_CLICK))
        }
        runCatching {
            val tone = ToneGenerator(AudioManager.STREAM_NOTIFICATION, 60)
            tone.startTone(ToneGenerator.TONE_PROP_ACK, 100)
            Thread {
                Thread.sleep(250)
                tone.release()
            }.start()
        }
    }
}
