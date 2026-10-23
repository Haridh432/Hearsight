package com.hsv2.hsv2_lite.voice

import android.content.Context
import android.os.SystemClock
import android.util.Log
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch
import org.json.JSONObject
import java.io.File
import java.util.ArrayDeque
import java.util.concurrent.Executors

enum class AssistantVoiceState {
    IDLE,
    LISTENING,
    PROCESSING,
    SPEAKING,
    ERROR
}

/**
 * Coordinates microphone capture, VAD, offline speech recognition (Moonshine & Parakeet),
 * and voice command execution for Hearsight.
 */
class VoiceAssistantManager(
    private val context: Context,
    private val commandProcessor: VoiceCommandProcessor,
    private val onSayResponse: (text: String) -> Unit
) {
    private val scope = CoroutineScope(SupervisorJob() + Dispatchers.Default)
    private val worker = Executors.newSingleThreadExecutor { Thread(it, "hearsight-voice-worker") }

    private val _state = MutableStateFlow(AssistantVoiceState.IDLE)
    val state: StateFlow<AssistantVoiceState> = _state.asStateFlow()

    private val _lastTranscript = MutableStateFlow("")
    val lastTranscript: StateFlow<String> = _lastTranscript.asStateFlow()

    private val _lastError = MutableStateFlow<String?>(null)
    val lastError: StateFlow<String?> = _lastError.asStateFlow()

    private val speechDir = File(context.getExternalFilesDir(null), "speech")
    private val vadFile = File(speechDir, "silero_vad.onnx")
    private val moonshineDir = File(speechDir, "moonshine/${MoonshineAsr.Size.TINY.label}")
    private val parakeetDir = File(speechDir, "parakeet-unified-en-0.6b-int8")

    @Volatile private var vad: SileroVad? = null
    @Volatile private var moonshine: MoonshineAsr? = null
    @Volatile private var parakeet: ParakeetAsr? = null
    @Volatile private var mic: MicCapture? = null

    private val audioRoute = AudioRoute(context)
    private var activeStream: MoonshineAsr.Stream? = null

    // VAD & turn tracking
    private var speechFrames = 0
    private var silenceFrames = 0
    private var hasSpoken = false
    private val recordedFrames = mutableListOf<FloatArray>()
    private val preRoll = ArrayDeque<FloatArray>(16)

    init {
        speechDir.mkdirs()
        initModelsInBackground()
    }

    private fun initModelsInBackground() {
        worker.execute {
            try {
                if (vadFile.exists()) {
                    runCatching {
                        vad = SileroVad(vadFile)
                        Log.i(TAG, "Silero VAD initialized.")
                    }.onFailure { Log.w(TAG, "Failed to load Silero VAD", it) }
                } else {
                    Log.i(TAG, "Silero VAD model not present at: ${vadFile.absolutePath}")
                }

                if (moonshineDir.exists()) {
                    runCatching {
                        val m = MoonshineAsr(MoonshineAsr.Size.TINY, moonshineDir)
                        if (m.isModelPresent()) {
                            m.load()
                            moonshine = m
                            Log.i(TAG, "Moonshine ASR initialized.")
                        }
                    }.onFailure { Log.w(TAG, "Failed to load Moonshine ASR", it) }
                }

                val p = ParakeetAsr(parakeetDir)
                if (p.isPresent()) {
                    runCatching {
                        p.load()
                        parakeet = p
                        Log.i(TAG, "Parakeet ASR initialized.")
                    }.onFailure { Log.w(TAG, "Failed to load Parakeet ASR", it) }
                }
            } catch (t: Throwable) {
                Log.e(TAG, "Voice model init error", t)
            }
        }
    }

    /**
     * Start listening for a voice command (Push-to-Talk or manual trigger).
     */
    fun startListening() {
        if (_state.value == AssistantVoiceState.LISTENING) return
        _state.value = AssistantVoiceState.LISTENING
        _lastError.value = null
        hasSpoken = false
        speechFrames = 0
        silenceFrames = 0
        recordedFrames.clear()

        Earcon.wake(context)
        audioRoute.begin()

        worker.execute {
            try {
                vad?.reset()
                moonshine?.let { m ->
                    activeStream = m.Stream { partial ->
                        _lastTranscript.value = partial
                    }
                }
                mic = MicCapture { frame ->
                    onAudioFrame(frame)
                }.also { it.start() }
            } catch (t: Throwable) {
                Log.e(TAG, "Failed to start listening", t)
                _state.value = AssistantVoiceState.ERROR
                _lastError.value = t.message
                stopListeningInternal()
            }
        }
    }

    /**
     * Stop listening manually and process the accumulated speech.
     */
    fun stopListening() {
        if (_state.value != AssistantVoiceState.LISTENING) return
        finishTurn()
    }

    fun toggleListening() {
        if (_state.value == AssistantVoiceState.LISTENING) {
            stopListening()
        } else {
            startListening()
        }
    }

    private fun onAudioFrame(frame: FloatArray) {
        if (_state.value != AssistantVoiceState.LISTENING) return

        recordedFrames.add(frame)
        activeStream?.push(frame)

        val p = vad?.probability(frame) ?: 0.5f
        if (p > 0.5f) {
            speechFrames++
            silenceFrames = 0
            if (speechFrames >= 3) {
                hasSpoken = true
            }
        } else {
            if (hasSpoken) {
                silenceFrames++
                // ~600 ms of silence ends the turn
                if (silenceFrames >= 18) {
                    finishTurn()
                }
            }
        }

        // Limit utterance duration to 15 seconds
        if (recordedFrames.size > 15 * (16000 / SileroVad.WINDOW)) {
            finishTurn()
        }
    }

    private fun finishTurn() {
        _state.value = AssistantVoiceState.PROCESSING
        stopListeningInternal()

        worker.execute {
            try {
                var transcript = activeStream?.finish()?.trim().orEmpty()
                activeStream?.close()
                activeStream = null

                // If Parakeet is available, decode the whole captured audio for higher accuracy
                parakeet?.let { p ->
                    val full = concatFrames(recordedFrames)
                    if (full.isNotEmpty()) {
                        val parakeetText = runCatching { p.transcribe(full) }.getOrNull()
                        if (!parakeetText.isNullOrBlank()) {
                            transcript = parakeetText
                        }
                    }
                }

                _lastTranscript.value = transcript
                Log.i(TAG, "Final recognized transcript: \"$transcript\"")

                if (transcript.isNotBlank()) {
                    val reply = commandProcessor.process(transcript)
                    if (reply != null) {
                        _state.value = AssistantVoiceState.SPEAKING
                        onSayResponse(reply)
                    }
                }
            } catch (t: Throwable) {
                Log.e(TAG, "Error processing voice turn", t)
                _lastError.value = t.message
            } finally {
                _state.value = AssistantVoiceState.IDLE
                audioRoute.end()
            }
        }
    }

    private fun stopListeningInternal() {
        mic?.stop()
        mic = null
    }

    private fun concatFrames(frames: List<FloatArray>): FloatArray {
        val total = frames.sumOf { it.size }
        val out = FloatArray(total)
        var offset = 0
        for (f in frames) {
            System.arraycopy(f, 0, out, offset, f.size)
            offset += f.size
        }
        return out
    }

    fun getStatusJson(): JSONObject {
        return JSONObject()
            .put("state", _state.value.name)
            .put("transcript", _lastTranscript.value)
            .put("error", _lastError.value ?: JSONObject.NULL)
            .put("vadReady", vad != null)
            .put("asrReady", moonshine != null || parakeet != null)
    }

    fun release() {
        stopListeningInternal()
        activeStream?.close()
        audioRoute.end()
        vad?.close()
        moonshine?.close()
        parakeet?.close()
    }

    companion object {
        private const val TAG = "HSV2VoiceManager"
    }
}
