package com.hsv2.hsv2_lite.voice

import com.k2fsa.sherpa.onnx.SileroVadModelConfig
import com.k2fsa.sherpa.onnx.Vad
import com.k2fsa.sherpa.onnx.VadModelConfig
import java.io.File

/** Silero VAD via sherpa-onnx. 16 kHz mono input. */
class SileroVad(
    model: File,
    threshold: Float = 0.5f,
    minSilenceSec: Float = 0.25f,
    minSpeechSec: Float = 0.25f,
) : AutoCloseable {

    private val windowSize = 512
    private val vad = Vad(
        null,
        VadModelConfig(
            sileroVadModelConfig = SileroVadModelConfig(
                model = model.absolutePath,
                threshold = threshold,
                minSilenceDuration = minSilenceSec,
                minSpeechDuration = minSpeechSec,
                windowSize = windowSize,
            ),
            sampleRate = 16000,
            numThreads = 1,
        ),
    )

    /** Seconds from the start of [pcm] to the end of the last detected speech, or null if none. */
    fun lastSpeechEnd(pcm: Pcm): Double? {
        val audio = pcm.resampled(16000)
        vad.reset()
        var lastEnd: Double? = null
        var i = 0
        while (i + windowSize <= audio.samples.size) {
            vad.acceptWaveform(audio.samples.copyOfRange(i, i + windowSize))
            i += windowSize
            while (!vad.empty()) {
                val seg = vad.front()
                lastEnd = (seg.start + seg.samples.size) / 16000.0
                vad.pop()
            }
        }
        vad.flush()
        while (!vad.empty()) {
            val seg = vad.front()
            lastEnd = (seg.start + seg.samples.size) / 16000.0
            vad.pop()
        }
        return lastEnd
    }

    /**
     * Speech probability (0..1) for one [WINDOW]-sample frame of 16 kHz audio. Streaming: the model keeps
     * state across calls, so call [reset] between utterances.
     */
    fun probability(frame: FloatArray): Float = vad.compute(frame)

    fun reset() = vad.reset()

    override fun close() = vad.release()

    companion object {
        /** Samples per frame for [probability]: 32 ms at 16 kHz. */
        const val WINDOW = 512
    }
}
