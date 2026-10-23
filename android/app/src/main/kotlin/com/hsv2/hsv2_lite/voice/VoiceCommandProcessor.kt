package com.hsv2.hsv2_lite.voice

import android.util.Log
import org.json.JSONObject
import java.util.Locale
import kotlin.math.abs

enum class VoiceIntent {
    START_NAVIGATION,
    STOP_NAVIGATION,
    DESCRIBE_SURROUNDINGS,
    WHAT_IN_FRONT,
    WHAT_ON_LEFT,
    WHAT_ON_RIGHT,
    READ_SCREEN,
    GREETING,
    HELP,
    UNKNOWN
}

data class ParsedCommand(
    val intent: VoiceIntent,
    val rawText: String,
    val cleanedText: String
)

/**
 * Parses user voice transcripts into actionable navigation commands and perception queries,
 * generating accurate spoken responses based solely on live telemetry data.
 */
class VoiceCommandProcessor(
    private val onStartNavigation: () -> String?, // returns error or null on success
    private val onStopNavigation: () -> Unit,
    private val onDescribeSurroundings: (question: String?) -> Unit,
    private val getTelemetry: () -> JSONObject?
) {

    fun process(transcript: String): String? {
        val parsed = parse(transcript)
        Log.i(TAG, "Parsed command: ${parsed.intent} from \"$transcript\"")
        return when (parsed.intent) {
            VoiceIntent.START_NAVIGATION -> {
                val err = onStartNavigation()
                if (err != null) "Could not start navigation: $err"
                else "Navigation started."
            }
            VoiceIntent.STOP_NAVIGATION -> {
                onStopNavigation()
                "Navigation stopped."
            }
            VoiceIntent.DESCRIBE_SURROUNDINGS -> {
                onDescribeSurroundings("briefly describe what is around me.")
                null // Handled asynchronously by SceneDescriber -> TTS
            }
            VoiceIntent.WHAT_IN_FRONT -> {
                queryFront()
            }
            VoiceIntent.WHAT_ON_LEFT -> {
                querySector(minAngle = -65.0, maxAngle = -15.0, sectorName = "left")
            }
            VoiceIntent.WHAT_ON_RIGHT -> {
                querySector(minAngle = 15.0, maxAngle = 65.0, sectorName = "right")
            }
            VoiceIntent.READ_SCREEN -> {
                queryScreenStatus()
            }
            VoiceIntent.GREETING -> {
                "Hello! I am Sago, your Hearsight voice assistant. How can I help you today?"
            }
            VoiceIntent.HELP -> {
                "You can ask me to: read screen, describe surroundings, start navigation, stop navigation, or ask what is in front, on the left, or on the right."
            }
            VoiceIntent.UNKNOWN -> {
                if (parsed.cleanedText.isBlank()) null
                else "I heard: \"${parsed.rawText}\". Say help, read screen, or start navigation."
            }
        }
    }

    fun parse(text: String): ParsedCommand {
        val clean = cleanTranscript(text)
        val intent = when {
            START_NAV_REGEX.containsMatchIn(clean) -> VoiceIntent.START_NAVIGATION
            STOP_NAV_REGEX.containsMatchIn(clean) -> VoiceIntent.STOP_NAVIGATION
            DESCRIBE_REGEX.containsMatchIn(clean) -> VoiceIntent.DESCRIBE_SURROUNDINGS
            READ_SCREEN_REGEX.containsMatchIn(clean) -> VoiceIntent.READ_SCREEN
            GREETING_REGEX.containsMatchIn(clean) -> VoiceIntent.GREETING
            HELP_REGEX.containsMatchIn(clean) -> VoiceIntent.HELP
            FRONT_REGEX.containsMatchIn(clean) -> VoiceIntent.WHAT_IN_FRONT
            LEFT_REGEX.containsMatchIn(clean) -> VoiceIntent.WHAT_ON_LEFT
            RIGHT_REGEX.containsMatchIn(clean) -> VoiceIntent.WHAT_ON_RIGHT
            else -> VoiceIntent.UNKNOWN
        }
        return ParsedCommand(intent, text, clean)
    }

    private fun cleanTranscript(text: String): String {
        return text.lowercase(Locale.ROOT)
            .replace(Regex("""^.*?\b(hey|hi|okay|ok)\s+(hearsight|argus|sago)\b[,.]*\s*"""), "")
            .replace(Regex("""\b(please|kindly|can you|could you|would you|tell me|show me)\b"""), "")
            .replace(Regex("""[^\w\s]"""), " ")
            .replace(Regex("""\s+"""), " ")
            .trim()
    }

    private fun queryFront(): String {
        val telemetry = getTelemetry() ?: return "Camera is not streaming."
        val status = telemetry.optString("status")
        if (status != "streaming") return "Camera is not active. Say start navigation to begin."

        val validator = telemetry.optJSONObject("validator")
        val safety = telemetry.optJSONObject("safety")
        val aheadDist = validator?.optDouble("ahead", -1.0) ?: -1.0
        val aheadSrc = validator?.optString("aheadSrc") ?: ""
        val obstacleDist = safety?.optDouble("obstacleDist", -1.0) ?: -1.0

        // Check front detections (|bearing_deg| <= 15)
        val dets = telemetry.optJSONArray("detections")
        val frontDets = mutableListOf<String>()
        if (dets != null) {
            for (i in 0 until dets.length()) {
                val d = dets.optJSONObject(i) ?: continue
                val bearing = d.optDouble("bearing_deg", 999.0)
                val dist = d.optDouble("distance", -1.0)
                val label = d.optString("name", d.optString("label", "object"))
                if (abs(bearing) <= 15.0 && dist in 0.2..6.0) {
                    frontDets.add("$label at %.1f meters".format(dist))
                }
            }
        }

        return when {
            frontDets.isNotEmpty() -> "In front: " + frontDets.take(2).joinToString(", ") + "."
            aheadDist in 0.3..5.0 -> {
                val srcLabel = if (aheadSrc.isNotBlank() && aheadSrc != "none") aheadSrc else "obstacle"
                "Obstacle in front at %.1f meters ($srcLabel).".format(aheadDist)
            }
            obstacleDist in 0.3..5.0 -> "Nearest obstacle ahead at %.1f meters.".format(obstacleDist)
            validator?.optBoolean("mayMove") == true -> "The path directly ahead is clear."
            else -> "No immediate obstacles detected straight ahead."
        }
    }

    private fun querySector(minAngle: Double, maxAngle: Double, sectorName: String): String {
        val telemetry = getTelemetry() ?: return "Camera is not streaming."
        val status = telemetry.optString("status")
        if (status != "streaming") return "Camera is not active. Say start navigation to begin."

        val dets = telemetry.optJSONArray("detections")
        val matches = mutableListOf<String>()
        if (dets != null) {
            for (i in 0 until dets.length()) {
                val d = dets.optJSONObject(i) ?: continue
                val bearing = d.optDouble("bearing_deg", 999.0)
                val dist = d.optDouble("distance", -1.0)
                val label = d.optString("name", d.optString("label", "object"))
                if (bearing in minAngle..maxAngle && dist in 0.3..5.0) {
                    matches.add("$label at %.1f meters".format(dist))
                }
            }
        }

        return if (matches.isNotEmpty()) {
            "On your $sectorName: " + matches.take(2).joinToString(", ") + "."
        } else {
            "Nothing detected on your $sectorName within range."
        }
    }

    private fun queryScreenStatus(): String {
        val telemetry = getTelemetry()
        val status = telemetry?.optString("status") ?: "idle"
        val isStreaming = status == "streaming"
        val fps = telemetry?.optDouble("fps", 0.0) ?: 0.0

        val cameraStatus = if (isStreaming) {
            "Camera is streaming at %.0f frames per second.".format(fps)
        } else {
            "Camera is disconnected or waiting for start."
        }

        val safety = telemetry?.optJSONObject("safety")
        val obstacleDist = safety?.optDouble("obstacleDist", -1.0) ?: -1.0
        val obstacleInfo = if (isStreaming) {
            if (obstacleDist in 0.3..5.0) {
                "Nearest obstacle is %.1f meters ahead.".format(obstacleDist)
            } else {
                "Path ahead is clear."
            }
        } else {
            ""
        }

        val dets = telemetry?.optJSONArray("detections")
        val count = dets?.length() ?: 0
        val detInfo = if (isStreaming && count > 0) {
            val names = mutableListOf<String>()
            for (i in 0 until minOf(count, 3)) {
                val d = dets!!.optJSONObject(i) ?: continue
                names.add(d.optString("name", d.optString("label", "object")))
            }
            "Detected on screen: ${names.joinToString(", ")}."
        } else ""

        val summary = listOf(cameraStatus, obstacleInfo, detInfo).filter { it.isNotBlank() }.joinToString(" ")
        return if (summary.isNotBlank()) "Screen status: $summary" else "Screen status: Idle, waiting for navigation."
    }

    companion object {
        private const val TAG = "HSV2VoiceCmd"

        private val START_NAV_REGEX = Regex("""\b(start|begin|resume)\s+(navigation|navigating|walk|walking|camera)\b""")
        private val STOP_NAV_REGEX = Regex("""\b(stop|halt|pause|end|cancel)\s+(navigation|navigating|walk|walking|camera)\b""")
        private val DESCRIBE_REGEX = Regex("""\b(describe|description|tell me about)\s+(the\s+)?(surroundings?|scene|room|environment|around)\b|\bwhat is around me\b|\bwhat'?s around me\b""")
        private val READ_SCREEN_REGEX = Regex("""\b(read|tell me|what is|what's|check|show)\s+(the\s+)?(screen|status|display|telemetry)\b|\bread\s+screen\b|\bscreen\s+status\b|\bwhat\s+does\s+the\s+screen\s+say\b""")
        private val FRONT_REGEX = Regex("""\bwhat\s+(is|s)\s+(in\s+front|ahead|directly\s+ahead)\b|\bwhat'?s\s+(in\s+front|ahead)\b""")
        private val LEFT_REGEX = Regex("""\bwhat\s+(is|s)\s+(on\s+(my\s+)?left|to\s+(my\s+)?left)\b|\b(look|anything\s+on\s+the)\s+left\b""")
        private val RIGHT_REGEX = Regex("""\bwhat\s+(is|s)\s+(on\s+(my\s+)?right|to\s+(my\s+)?right)\b|\b(look|anything\s+on\s+the)\s+right\b""")
        private val GREETING_REGEX = Regex("""\b(hello|hi|hey|good\s+(morning|afternoon|evening)|who\s+are\s+you|who\s+is\s+this)\b""")
        private val HELP_REGEX = Regex("""\b(help|what\s+can\s+you\s+do|commands|options|how\s+to\s+use)\b""")
    }
}
