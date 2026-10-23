package com.hsv2.hsv2_lite.voice

import android.content.Context
import android.media.AudioAttributes
import android.media.AudioDeviceInfo
import android.media.AudioFocusRequest
import android.media.AudioManager

/**
 * Puts audio routing into communication mode during voice interactions, allowing the hardware
 * AEC to operate effectively.
 */
class AudioRoute(context: Context) {

    private val am = context.getSystemService(AudioManager::class.java)
    private var focus: AudioFocusRequest? = null
    private var active = false

    val attributes: AudioAttributes = AudioAttributes.Builder()
        .setUsage(AudioAttributes.USAGE_VOICE_COMMUNICATION)
        .setContentType(AudioAttributes.CONTENT_TYPE_SPEECH)
        .build()

    @Synchronized
    fun begin() {
        if (active) return
        active = true
        focus = AudioFocusRequest.Builder(AudioManager.AUDIOFOCUS_GAIN_TRANSIENT_EXCLUSIVE)
            .setAudioAttributes(attributes)
            .build()
            .also { am?.requestAudioFocus(it) }
        am?.mode = AudioManager.MODE_IN_COMMUNICATION
        val devices = am?.availableCommunicationDevices.orEmpty()
        val preferred = PREFERENCE.firstNotNullOfOrNull { type -> devices.firstOrNull { it.type == type } }
        preferred?.let { am?.setCommunicationDevice(it) }
    }

    @Synchronized
    fun end() {
        if (!active) return
        active = false
        am?.clearCommunicationDevice()
        am?.mode = AudioManager.MODE_NORMAL
        focus?.let { am?.abandonAudioFocusRequest(it) }
        focus = null
    }

    private companion object {
        val PREFERENCE = listOf(
            AudioDeviceInfo.TYPE_BLE_HEADSET,
            AudioDeviceInfo.TYPE_BLUETOOTH_SCO,
            AudioDeviceInfo.TYPE_USB_HEADSET,
            AudioDeviceInfo.TYPE_WIRED_HEADSET,
            AudioDeviceInfo.TYPE_BUILTIN_SPEAKER,
        )
    }
}
