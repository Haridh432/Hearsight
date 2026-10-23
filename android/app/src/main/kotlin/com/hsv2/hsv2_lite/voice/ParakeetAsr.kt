package com.hsv2.hsv2_lite.voice

import com.k2fsa.sherpa.onnx.OfflineModelConfig
import com.k2fsa.sherpa.onnx.OfflineRecognizer
import com.k2fsa.sherpa.onnx.OfflineRecognizerConfig
import com.k2fsa.sherpa.onnx.OfflineTransducerModelConfig
import java.io.File

/**
 * NVIDIA Parakeet Unified EN 0.6B (int8, sherpa-onnx export).
 * Runs on a finished utterance for final punctuated transcription.
 * [dir] holds encoder.int8.onnx, decoder.int8.onnx, joiner.int8.onnx and tokens.txt.
 */
class ParakeetAsr(private val dir: File, private val threads: Int = 4) : AutoCloseable {

    private var recognizer: OfflineRecognizer? = null

    fun isPresent(): Boolean = FILES.all { File(dir, it).exists() }

    /** Returns load seconds. Blocking; call off the main thread. */
    fun load(): Double {
        val t0 = System.nanoTime()
        recognizer = OfflineRecognizer(
            config = OfflineRecognizerConfig(
                modelConfig = OfflineModelConfig(
                    transducer = OfflineTransducerModelConfig(
                        encoder = path(ENCODER),
                        decoder = path(DECODER),
                        joiner = path(JOINER)
                    ),
                    tokens = path(TOKENS),
                    numThreads = threads,
                    provider = "cpu",
                    modelType = "nemo_transducer",
                ),
            ),
        )
        return (System.nanoTime() - t0) / 1e9
    }

    /** The transcript of 16 kHz mono [samples]. */
    @Synchronized
    fun transcribe(samples: FloatArray, sampleRate: Int = 16000): String {
        val r = recognizer ?: error("Parakeet isn't loaded")
        val stream = r.createStream()
        try {
            stream.acceptWaveform(samples, sampleRate)
            r.decode(stream)
            return r.getResult(stream).text.trim()
        } finally {
            stream.release()
        }
    }

    override fun close() {
        recognizer?.release()
        recognizer = null
    }

    private fun path(name: String) = File(dir, name).absolutePath

    companion object {
        const val ENCODER = "encoder.int8.onnx"
        const val DECODER = "decoder.int8.onnx"
        const val JOINER = "joiner.int8.onnx"
        const val TOKENS = "tokens.txt"
        val FILES = listOf(ENCODER, DECODER, JOINER, TOKENS)
    }
}
