package com.hsv2.hsv2_lite.voice

import ai.moonshine.voice.AssetDownloader
import ai.moonshine.voice.JNI
import ai.moonshine.voice.ModelSpec
import ai.moonshine.voice.Transcriber
import ai.moonshine.voice.TranscriptEvent
import ai.moonshine.voice.TranscriptEventListener
import java.io.File
import java.util.concurrent.TimeUnit

/** Moonshine streaming ASR (English). Model files live in [modelDir]. */
class MoonshineAsr(val size: Size = Size.TINY, private val modelDir: File) : AutoCloseable {

    enum class Size(val arch: Int, val label: String) {
        TINY(JNI.MOONSHINE_MODEL_ARCH_TINY_STREAMING, "tiny-streaming"),
        SMALL(JNI.MOONSHINE_MODEL_ARCH_SMALL_STREAMING, "small-streaming"),
        MEDIUM(JNI.MOONSHINE_MODEL_ARCH_MEDIUM_STREAMING, "medium-streaming"),
    }

    private val transcriber = Transcriber()

    /** Downloads the model files once (network) if missing. Blocking; call off the main thread. */
    fun ensureModel(onProgress: (String) -> Unit = {}) {
        val spec = ModelSpec.stt("en", size.arch, false)
        val downloader = AssetDownloader()
        if (!downloader.isModelPresent(modelDir, spec)) {
            downloader.ensureModelPresent(modelDir, spec) { path, index, total, done, bytes ->
                onProgress("$path $index/$total ${done / 1_000_000}/${bytes / 1_000_000} MB")
            }
        }
    }

    /** Downloads the model files (network), reporting overall progress 0..1. Blocking. */
    fun download(onProgress: (Float) -> Unit) {
        val spec = ModelSpec.stt("en", size.arch, false)
        AssetDownloader().ensureModelPresent(modelDir, spec) { _, index, total, done, bytes ->
            val file = if (bytes > 0) done.toFloat() / bytes else 0f
            onProgress(((index - 1).coerceAtLeast(0) + file) / total.coerceAtLeast(1))
        }
    }

    /** True when the model files are already on the phone (no network). */
    fun isModelPresent(): Boolean = AssetDownloader().isModelPresent(modelDir, ModelSpec.stt("en", size.arch, false))

    /** Returns load seconds. */
    fun load(): Double {
        val t0 = System.nanoTime()
        transcriber.loadFromFiles(modelDir.absolutePath, size.arch)
        return (System.nanoTime() - t0) / 1e9
    }

    /**
     * Words recognition should lean toward, such as the assistant's name.
     */
    fun setKeyterms(terms: List<String>) = transcriber.setKeyterms(terms)

    /** Seconds of audio between transcription passes (library default 0.5). */
    var updateInterval: Double
        get() = transcriber.updateInterval
        set(value) { transcriber.updateInterval = value }

    data class Result(
        val text: String,
        val finalLatencyMs: Double,
        val stopMs: Double,
        val engineLatencyMs: Int?,
        val audioSeconds: Double,
        val wallSeconds: Double,
    )

    fun transcribe(pcm: Pcm, chunkMs: Int = 80, realtime: Boolean = true, feedSeconds: Double? = null): Result {
        val full = pcm.resampled(16000)
        val audio = feedSeconds?.let { s ->
            val n = (s * full.sampleRate).toInt().coerceIn(1, full.samples.size)
            Pcm(full.samples.copyOf(n), full.sampleRate)
        } ?: full
        val lines = sortedMapOf<Long, String>()
        val stream = transcriber.createStream()
        var lastLatency: Int? = null
        val listener = java.util.function.Consumer<TranscriptEvent> { ev ->
            ev.accept(object : TranscriptEventListener() {
                override fun onLineTextChanged(e: TranscriptEvent.LineTextChanged) {
                    if (e.streamHandle == stream) synchronized(lines) { lines[e.line.id] = e.line.text }
                }
                override fun onLineCompleted(e: TranscriptEvent.LineCompleted) {
                    if (e.streamHandle == stream) synchronized(lines) {
                        lines[e.line.id] = e.line.text
                        lastLatency = e.line.lastTranscriptionLatencyMs
                    }
                }
            })
        }
        transcriber.addListener(listener)
        try {
            val t0 = System.nanoTime()
            transcriber.startStream(stream)
            val chunk = audio.sampleRate * chunkMs / 1000
            var i = 0
            while (i < audio.samples.size) {
                val end = minOf(i + chunk, audio.samples.size)
                transcriber.addAudioToStream(stream, audio.samples.copyOfRange(i, end), audio.sampleRate)
                i = end
                if (realtime) {
                    val due = t0 + (i.toLong() * 1_000_000_000L / audio.sampleRate)
                    val wait = due - System.nanoTime()
                    if (wait > 0) TimeUnit.NANOSECONDS.sleep(wait)
                }
            }
            val scheduledEnd = t0 + (audio.seconds * 1e9).toLong()
            val stopStart = System.nanoTime()
            transcriber.stopStream(stream)
            val finished = System.nanoTime()
            val text = synchronized(lines) { lines.values.joinToString(" ") { it.trim() }.trim() }
            val anchor = if (realtime) scheduledEnd else stopStart
            return Result(text, (finished - anchor) / 1e6, (finished - stopStart) / 1e6, lastLatency, audio.seconds, (finished - t0) / 1e9)
        } finally {
            transcriber.removeListener(listener)
            transcriber.freeStream(stream)
        }
    }

    /**
     * A live transcription stream: push audio as it arrives, receive partial transcripts via [onText],
     * and call [finish] when speech ends.
     */
    inner class Stream(onText: (String) -> Unit = {}) : AutoCloseable {
        @Volatile var onText: (String) -> Unit = onText
        private val handle = transcriber.createStream()
        private val lines = sortedMapOf<Long, String>()
        private val listener = java.util.function.Consumer<TranscriptEvent> { ev ->
            ev.accept(object : TranscriptEventListener() {
                override fun onLineTextChanged(e: TranscriptEvent.LineTextChanged) {
                    if (e.streamHandle == handle) update(e.line.id, e.line.text)
                }
                override fun onLineCompleted(e: TranscriptEvent.LineCompleted) {
                    if (e.streamHandle == handle) update(e.line.id, e.line.text)
                }
            })
        }

        init {
            transcriber.addListener(listener)
            transcriber.startStream(handle)
        }

        private fun update(id: Long, text: String) {
            val all = synchronized(lines) {
                lines[id] = text
                lines.values.joinToString(" ") { it.trim() }.trim()
            }
            onText(all)
        }

        fun push(samples: FloatArray, sampleRate: Int = 16000) = transcriber.addAudioToStream(handle, samples, sampleRate)

        fun finish(): String {
            transcriber.stopStream(handle)
            return synchronized(lines) { lines.values.joinToString(" ") { it.trim() }.trim() }
        }

        override fun close() {
            transcriber.removeListener(listener)
            transcriber.freeStream(handle)
        }
    }

    override fun close() = transcriber.close()
}
