package com.hsv2.hsv2_lite.voice

import android.Manifest
import android.content.Context
import android.content.Intent
import android.content.pm.PackageManager
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.speech.RecognitionListener
import android.speech.RecognizerIntent
import android.speech.SpeechRecognizer
import android.util.Log
import androidx.core.content.ContextCompat
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import org.json.JSONObject

enum class AssistantVoiceState {
    IDLE,
    LISTENING,
    PROCESSING,
    SPEAKING,
    ERROR
}

/**
 * Coordinates Google Assistant / Android SpeechRecognizer speech-to-text,
 * live partial transcripts, and voice command execution for Hearsight.
 */
class VoiceAssistantManager(
    private val context: Context,
    private val commandProcessor: VoiceCommandProcessor,
    private val onSayResponse: (text: String) -> Unit
) {
    private val mainHandler = Handler(Looper.getMainLooper())

    private val _state = MutableStateFlow(AssistantVoiceState.IDLE)
    val state: StateFlow<AssistantVoiceState> = _state.asStateFlow()

    private val _lastTranscript = MutableStateFlow("")
    val lastTranscript: StateFlow<String> = _lastTranscript.asStateFlow()

    private val _lastError = MutableStateFlow<String?>(null)
    val lastError: StateFlow<String?> = _lastError.asStateFlow()

    private var speechRecognizer: SpeechRecognizer? = null
    private val audioRoute = AudioRoute(context)
    private var pendingListenRunnable: Runnable? = null

    init {
        mainHandler.post {
            ensureSpeechRecognizer()
        }
    }

    private fun ensureSpeechRecognizer(): SpeechRecognizer? {
        if (speechRecognizer == null) {
            if (SpeechRecognizer.isRecognitionAvailable(context)) {
                try {
                    speechRecognizer = SpeechRecognizer.createSpeechRecognizer(context).apply {
                        setRecognitionListener(createRecognitionListener())
                    }
                    Log.i(TAG, "Google SpeechRecognizer created successfully")
                } catch (t: Throwable) {
                    Log.e(TAG, "Failed to create SpeechRecognizer", t)
                }
            } else {
                Log.w(TAG, "SpeechRecognizer is NOT available on this device")
            }
        }
        return speechRecognizer
    }

    /**
     * Start the assistant greeting interaction and automatically transition into listening mode
     * just like Google Assistant.
     */
    fun startInteraction(greeting: String? = "Hello! I am Sago. How can I help you?") {
        if (ContextCompat.checkSelfPermission(context, Manifest.permission.RECORD_AUDIO)
            != PackageManager.PERMISSION_GRANTED
        ) {
            Log.w(TAG, "Cannot start voice interaction: RECORD_AUDIO permission missing")
            return
        }

        // Ignore duplicate start requests if already active
        if (_state.value == AssistantVoiceState.SPEAKING || _state.value == AssistantVoiceState.LISTENING) {
            Log.i(TAG, "Voice interaction already in progress (${_state.value.name}), skipping duplicate start")
            return
        }

        pendingListenRunnable?.let { mainHandler.removeCallbacks(it) }

        if (!greeting.isNullOrBlank()) {
            _state.value = AssistantVoiceState.SPEAKING
            _lastTranscript.value = greeting
            onSayResponse(greeting)

            // Deliver spoken greeting, then immediately start listening with wake earcon
            val greetingDurationMs = (greeting.length * 65L).coerceIn(1500L, 2500L)
            val runnable = Runnable {
                pendingListenRunnable = null
                startListening()
            }
            pendingListenRunnable = runnable
            mainHandler.postDelayed(runnable, greetingDurationMs)
        } else {
            startListening()
        }
    }

    /**
     * Start listening for a voice command using Google Assistant speech recognition.
     */
    fun startListening() {
        if (ContextCompat.checkSelfPermission(context, Manifest.permission.RECORD_AUDIO)
            != PackageManager.PERMISSION_GRANTED
        ) {
            Log.w(TAG, "Cannot start listening: RECORD_AUDIO permission missing")
            _state.value = AssistantVoiceState.ERROR
            _lastError.value = "Microphone permission required"
            return
        }

        if (_state.value == AssistantVoiceState.LISTENING) {
            Log.d(TAG, "Already listening, skipping redundant startListening")
            return
        }

        pendingListenRunnable?.let {
            mainHandler.removeCallbacks(it)
            pendingListenRunnable = null
        }

        mainHandler.post {
            try {
                val recognizer = ensureSpeechRecognizer()
                if (recognizer == null) {
                    _state.value = AssistantVoiceState.ERROR
                    _lastError.value = "Speech recognition unavailable"
                    Log.e(TAG, "SpeechRecognizer is null")
                    return@post
                }

                // Safely cancel any lingering session
                try { recognizer.cancel() } catch (_: Exception) {}

                _state.value = AssistantVoiceState.LISTENING
                _lastError.value = null
                _lastTranscript.value = ""

                Earcon.wake(context)
                audioRoute.begin()

                val langPref = context.getSharedPreferences("sailens_voice", Context.MODE_PRIVATE)
                    .getString("lang", "en") ?: "en"
                val langTag = if (langPref == "ta") "ta-IN" else "en-US"

                val intent = Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH).apply {
                    putExtra(RecognizerIntent.EXTRA_LANGUAGE_MODEL, RecognizerIntent.LANGUAGE_MODEL_FREE_FORM)
                    putExtra(RecognizerIntent.EXTRA_PARTIAL_RESULTS, true)
                    putExtra(RecognizerIntent.EXTRA_MAX_RESULTS, 3)
                    putExtra(RecognizerIntent.EXTRA_CALLING_PACKAGE, context.packageName)
                    putExtra(RecognizerIntent.EXTRA_LANGUAGE, langTag)
                    putExtra(RecognizerIntent.EXTRA_LANGUAGE_PREFERENCE, langTag)
                }

                recognizer.startListening(intent)
                Log.i(TAG, "SpeechRecognizer listening ($langTag)")
            } catch (t: Throwable) {
                Log.e(TAG, "Failed to start listening", t)
                _state.value = AssistantVoiceState.ERROR
                _lastError.value = t.message
                audioRoute.end()
            }
        }
    }

    fun stopListening() {
        pendingListenRunnable?.let {
            mainHandler.removeCallbacks(it)
            pendingListenRunnable = null
        }
        mainHandler.post {
            try {
                speechRecognizer?.stopListening()
            } catch (e: Exception) {
                Log.w(TAG, "stopListening error", e)
            }
            audioRoute.end()
            if (_state.value == AssistantVoiceState.LISTENING || _state.value == AssistantVoiceState.SPEAKING) {
                _state.value = AssistantVoiceState.IDLE
            }
        }
    }

    fun toggleListening() {
        if (_state.value == AssistantVoiceState.LISTENING) {
            stopListening()
        } else {
            startListening()
        }
    }

    private fun createRecognitionListener(): RecognitionListener {
        return object : RecognitionListener {
            override fun onReadyForSpeech(params: Bundle?) {
                Log.i(TAG, "SpeechRecognizer: onReadyForSpeech")
                _state.value = AssistantVoiceState.LISTENING
            }

            override fun onBeginningOfSpeech() {
                Log.i(TAG, "SpeechRecognizer: onBeginningOfSpeech")
            }

            override fun onRmsChanged(rmsdB: Float) {}

            override fun onBufferReceived(buffer: ByteArray?) {}

            override fun onEndOfSpeech() {
                Log.i(TAG, "SpeechRecognizer: onEndOfSpeech")
                _state.value = AssistantVoiceState.PROCESSING
            }

            override fun onError(error: Int) {
                val errMsg = getSpeechErrorString(error)
                Log.w(TAG, "SpeechRecognizer onError: code=$error ($errMsg)")
                audioRoute.end()
                _state.value = AssistantVoiceState.IDLE
                if (error != SpeechRecognizer.ERROR_NO_MATCH && error != SpeechRecognizer.ERROR_SPEECH_TIMEOUT) {
                    _lastError.value = errMsg
                }
            }

            override fun onResults(results: Bundle?) {
                audioRoute.end()
                val matches = results?.getStringArrayList(SpeechRecognizer.RESULTS_RECOGNITION)
                val transcript = matches?.firstOrNull()?.trim().orEmpty()
                Log.i(TAG, "SpeechRecognizer onResults: \"$transcript\" (all: $matches)")
                _lastTranscript.value = transcript

                if (transcript.isNotBlank()) {
                    val reply = commandProcessor.process(transcript)
                    if (reply != null) {
                        _state.value = AssistantVoiceState.SPEAKING
                        onSayResponse(reply)
                        val speakDuration = (reply.length * 65L).coerceIn(1500L, 4500L)
                        mainHandler.postDelayed({
                            if (_state.value == AssistantVoiceState.SPEAKING) {
                                _state.value = AssistantVoiceState.IDLE
                            }
                        }, speakDuration)
                    } else {
                        _state.value = AssistantVoiceState.IDLE
                    }
                } else {
                    _state.value = AssistantVoiceState.IDLE
                }
            }

            override fun onPartialResults(partialResults: Bundle?) {
                val matches = partialResults?.getStringArrayList(SpeechRecognizer.RESULTS_RECOGNITION)
                val partial = matches?.firstOrNull()?.trim().orEmpty()
                if (partial.isNotBlank()) {
                    _lastTranscript.value = partial
                    Log.d(TAG, "SpeechRecognizer partial: \"$partial\"")
                }
            }

            override fun onEvent(eventType: Int, params: Bundle?) {}
        }
    }

    private fun getSpeechErrorString(error: Int): String {
        return when (error) {
            SpeechRecognizer.ERROR_AUDIO -> "Audio recording error"
            SpeechRecognizer.ERROR_CLIENT -> "Client side error"
            SpeechRecognizer.ERROR_INSUFFICIENT_PERMISSIONS -> "Insufficient permissions"
            SpeechRecognizer.ERROR_NETWORK -> "Network error"
            SpeechRecognizer.ERROR_NETWORK_TIMEOUT -> "Network timeout"
            SpeechRecognizer.ERROR_NO_MATCH -> "No speech match"
            SpeechRecognizer.ERROR_RECOGNIZER_BUSY -> "Recognition service busy"
            SpeechRecognizer.ERROR_SERVER -> "Server error"
            SpeechRecognizer.ERROR_SPEECH_TIMEOUT -> "No speech input"
            else -> "Speech recognition error ($error)"
        }
    }

    fun getStatusJson(): JSONObject {
        return JSONObject()
            .put("state", _state.value.name)
            .put("transcript", _lastTranscript.value)
            .put("error", _lastError.value ?: JSONObject.NULL)
            .put("vadReady", true)
            .put("asrReady", speechRecognizer != null || SpeechRecognizer.isRecognitionAvailable(context))
    }

    fun release() {
        pendingListenRunnable?.let {
            mainHandler.removeCallbacks(it)
            pendingListenRunnable = null
        }
        mainHandler.removeCallbacksAndMessages(null)
        mainHandler.post {
            try {
                speechRecognizer?.destroy()
                speechRecognizer = null
            } catch (e: Exception) {
                Log.w(TAG, "speechRecognizer.destroy error", e)
            }
            audioRoute.end()
            _state.value = AssistantVoiceState.IDLE
        }
    }

    companion object {
        private const val TAG = "HSV2VoiceManager"
    }
}
