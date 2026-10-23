package com.hsv2.hsv2_lite

import android.content.Context
import android.graphics.Bitmap
import android.os.SystemClock
import android.util.Log
import com.google.ai.edge.litertlm.Backend
import com.google.ai.edge.litertlm.Content
import com.google.ai.edge.litertlm.Contents
import com.google.ai.edge.litertlm.Engine
import com.google.ai.edge.litertlm.EngineConfig
import java.io.ByteArrayOutputStream
import java.io.File
import java.util.concurrent.Executors

/**
 * "What's ahead?" (Option 2): one short spoken description of the scene from Gemma 4 E2B,
 * running fully on the phone with LiteRT-LM (GPU), on its own thread.
 *
 * It only DESCRIBES. Safety decisions stay with the native safety validator: the caller
 * passes HearSight's facts into the prompt and filters the answer before speaking it
 * (MainActivity.speakDescription). Nothing here can make the user move.
 *
 * Model file: <app external files>/models/gemma-4-E2B-it.litertlm (copied from the
 * AI Edge Gallery download; not in the APK).
 */
class SceneDescriber(context: Context) {

    private val modelFile = File(context.getExternalFilesDir(null), "models/gemma-4-E2B-it.litertlm")
    private val cacheDir = context.cacheDir.absolutePath
    private val worker = Executors.newSingleThreadExecutor { r -> Thread(r, "scene-describer").apply { priority = Thread.NORM_PRIORITY - 1 } }
    @Volatile private var engine: Engine? = null
    @Volatile var busy = false
        private set
    @Volatile var lastInitMs = 0L
        private set

    val modelAvailable: Boolean get() = modelFile.isFile

    private fun ensureEngine(): Engine? {
        engine?.let { return it }
        if (!modelFile.isFile) {
            Log.w(TAG, "model missing: ${modelFile.absolutePath}")
            return null
        }
        val t0 = SystemClock.elapsedRealtime()
        return try {
            // Experiment switch: an empty file models/use_cpu selects the CPU backend
            // (adb shell touch .../files/models/use_cpu). Phone test: Gemma on the GPU
            // paused the camera loop > 0.7 s ("Camera stalled").
            val cpu = File(modelFile.parentFile, "use_cpu").exists()
            Log.i(TAG, "SCENE,backend,${if (cpu) "cpu" else "gpu"}")
            val e = Engine(
                EngineConfig(
                    modelPath = modelFile.absolutePath,
                    backend = if (cpu) Backend.CPU() else Backend.GPU(),
                    visionBackend = if (cpu) Backend.CPU() else Backend.GPU(),
                    cacheDir = cacheDir,
                )
            )
            e.initialize()
            lastInitMs = SystemClock.elapsedRealtime() - t0
            Log.i(TAG, "SCENE,init,${lastInitMs} ms")
            engine = e
            e
        } catch (t: Throwable) {
            Log.e(TAG, "engine init failed", t)
            null
        }
    }

    /** Loads the model in the background (first answer is then faster). */
    fun warmUp() = worker.execute { ensureEngine() }

    /**
     * Describes [rgb] (w*h*3) with HearSight's [facts] (may be empty). [onResult] gets the
     * sentence (or null on failure) and the time in ms, on the describer thread.
     */
    /**
     * [question]: what to ask (live navigation events ask focused questions); null = the
     * general "what is ahead" question.
     */
    fun describe(rgb: ByteArray, w: Int, h: Int, facts: String, question: String? = null,
                 onResult: (String?, Long) -> Unit) {
        if (busy) return
        busy = true
        worker.execute {
            val t0 = SystemClock.elapsedRealtime()
            var text: String? = null
            try {
                val e = ensureEngine()
                if (e != null) {
                    val jpeg = toJpeg(rgb, w, h)
                    val conv = e.createConversation()
                    try {
                        val reply = conv.sendMessage(
                            Contents.of(Content.ImageBytes(jpeg), Content.Text(prompt(facts, question)))
                        )
                        val raw = reply.contents.contents.filterIsInstance<Content.Text>().joinToString(" ") { it.text }
                        text = clean(raw)
                    } finally {
                        try { conv.close() } catch (_: Throwable) {}
                    }
                }
            } catch (t: Throwable) {
                Log.e(TAG, "describe failed", t)
            }
            val ms = SystemClock.elapsedRealtime() - t0
            Log.i(TAG, "SCENE,answer,$ms ms,${text ?: "-"}")
            busy = false
            onResult(text, ms)
        }
    }

    fun release() {
        worker.execute {
            try { engine?.close() } catch (_: Throwable) {}
            engine = null
        }
        worker.shutdown()
    }

    companion object {
        private const val TAG = "HSV2Scene"

        fun prompt(facts: String, question: String? = null): String =
            "I am blind and walking forward. Looking at this photo from my head-mounted camera, " +
                (question ?: ("tell me in ONE short sentence (at most 20 words) what is ahead of me and on my left " +
                    "and right: obstacles, doors, stairs, people, and whether the way ahead looks clear.")) +
                " Answer in ONE short sentence (at most 20 words). " +
                "Do not mention colours or the camera. Only say what you can see." +
                (if (facts.isNotBlank()) "\nMy sensors measured: $facts. Do not contradict them." else "")

        /** One sentence, no markdown, at most ~25 words. */
        fun clean(raw: String): String {
            var s = raw.replace(Regex("[*_#`>]"), " ").replace(Regex("\\s+"), " ").trim()
            // Sentence end: . ! ? followed by a space (not the point in "2.1 m").
            val end = Regex("[.!?](?=\\s)").find(s)?.range?.first ?: -1
            if (end >= 10) s = s.substring(0, end + 1)
            val words = s.split(" ")
            if (words.size > 25) s = words.take(25).joinToString(" ") + "."
            return s
        }

        fun toJpeg(rgb: ByteArray, w: Int, h: Int): ByteArray {
            val px = IntArray(w * h)
            for (i in 0 until w * h) {
                val r = rgb[i * 3].toInt() and 0xff
                val g = rgb[i * 3 + 1].toInt() and 0xff
                val b = rgb[i * 3 + 2].toInt() and 0xff
                px[i] = (0xff shl 24) or (r shl 16) or (g shl 8) or b
            }
            val bmp = Bitmap.createBitmap(px, w, h, Bitmap.Config.ARGB_8888)
            val out = ByteArrayOutputStream()
            bmp.compress(Bitmap.CompressFormat.JPEG, 90, out)
            bmp.recycle()
            return out.toByteArray()
        }
    }
}
