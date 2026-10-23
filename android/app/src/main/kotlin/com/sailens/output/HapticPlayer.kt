// Copied unchanged from Sailens Android (Apache-2.0), see third_party/sailens.
package com.sailens.output

import android.content.Context
import android.media.AudioAttributes
import android.os.Build
import android.os.VibrationAttributes
import android.os.VibrationEffect
import android.os.Vibrator
import android.os.VibratorManager

/**
 * The haptic primitive: plays a timing pattern and cancels it.
 *
 * Deliberately vocabulary-free. Which rhythm means "obstacle on your left" is a Guidance
 * decision, and that vocabulary stays in the shell's Guidance package (architecture.md §6.6).
 * What lives here is the part that is the same for any product: reaching the vibrator, coping
 * with motors that have no amplitude control, and asking for the accessibility usage so the
 * pattern still fires when the phone is silenced.
 */
public class HapticPlayer(context: Context) {

    private val vibrator: Vibrator? =
        (context.getSystemService(Context.VIBRATOR_MANAGER_SERVICE) as? VibratorManager)?.defaultVibrator

    /**
     * @param timings alternating off/on durations; index 0 is the initial delay, so even indices
     *   do not vibrate.
     * @param amplitude 1..255, applied to the on segments when the motor supports it.
     */
    public fun play(timings: LongArray, amplitude: Int) {
        val device = vibrator ?: return
        if (!device.hasVibrator()) return

        val effect = if (device.hasAmplitudeControl()) {
            val amplitudes = IntArray(timings.size) { index ->
                if (index % 2 == 0) 0 else amplitude
            }
            VibrationEffect.createWaveform(timings, amplitudes, NO_REPEAT)
        } else {
            // Amplitude tiers are meaningless on a motor without amplitude control; fall back to
            // the rhythm alone, which is where the information actually lives.
            VibrationEffect.createWaveform(timings, NO_REPEAT)
        }

        vibrate(device, effect)
    }

    public fun cancel() {
        vibrator?.cancel()
    }

    /**
     * Vibrate with the accessibility usage, so it still fires under Do Not Disturb or silent mode.
     *
     * A navigation cue is not a notification. Someone in a meeting or out at night has *more*
     * reason to be relying on the haptic channel, not less.
     */
    private fun vibrate(device: Vibrator, effect: VibrationEffect) {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            device.vibrate(
                effect,
                VibrationAttributes.createForUsage(VibrationAttributes.USAGE_ACCESSIBILITY),
            )
        } else {
            @Suppress("DEPRECATION")
            device.vibrate(
                effect,
                AudioAttributes.Builder()
                    .setUsage(AudioAttributes.USAGE_ASSISTANCE_ACCESSIBILITY)
                    .setContentType(AudioAttributes.CONTENT_TYPE_SONIFICATION)
                    .build(),
            )
        }
    }

    private companion object {
        const val NO_REPEAT = -1
    }
}
