package com.hsv2.hsv2_lite

import android.content.Context
import android.os.SystemClock
import android.util.Log
import com.sailens.core.log.LogService
import com.sailens.output.AccessibilityStatusProvider
import com.sailens.output.Announcement
import com.sailens.output.HapticPlayer
import com.sailens.output.SpeechManager
import org.json.JSONObject
import java.util.Locale

/**
 * Phase 6: speaks the Phase 5 guidance instruction and plays its haptic cue.
 *
 * Output mechanism (TTS with audio focus, strict-priority interruption, expiry; accessibility
 * vibration) is Sailens' (com.sailens.output). The policy here — what to say, in English or
 * Tamil, how often — follows the Sailens CooldownManager rules:
 *  - each instruction is spoken once when it changes; a repeat of the same instruction waits for
 *    its cooldown,
 *  - a more urgent instruction bypasses any cooldown and interrupts speech in progress
 *    (SpeechManager: strictly higher priority interrupts),
 *  - stop-type instructions and camera faults are repeated while they last, so the user is never
 *    left believing it is safe because the voice went quiet.
 * Speech that missed its moment (1.5 s) is dropped instead of spoken late.
 */
class VoiceGuide(context: Context) {

    private val logger = object : LogService {
        override fun debug(tag: String, message: String, data: Map<String, Any>?) {}
        override fun info(tag: String, message: String, data: Map<String, Any>?) {
            Log.i(TAG, "$tag: $message ${data ?: ""}")
        }
        override fun warning(tag: String, message: String, data: Map<String, Any>?, throwable: Throwable?) {
            Log.w(TAG, "$tag: $message ${data ?: ""}", throwable)
        }
        override fun error(tag: String, message: String, throwable: Throwable?) {
            Log.e(TAG, "$tag: $message", throwable)
        }
    }

    private val speech = SpeechManager(context.applicationContext, logger)
    private val haptics = HapticPlayer(context.applicationContext)
    // TalkBack (screen reader) on? Guidance speech stays on our TTS either way (it needs strict
    // priority interruption); the Flutter banners are not live regions, so TalkBack does not read
    // the same instruction a second time.
    private val a11y = AccessibilityStatusProvider(context.applicationContext)
    private val prefs = context.getSharedPreferences("hearsight_voice", Context.MODE_PRIVATE)

    /** Called with every message actually spoken (Phase 7 validation log). */
    var onSay: ((key: String, text: String) -> Unit)? = null

    var enabled: Boolean = prefs.getBoolean("enabled", true)
        private set
    /** "en", "ta" or "both" (Tamil voice, then English voice) as chosen; [spokenLang] is what is actually spoken. */
    var language: String = prefs.getString("lang", "en") ?: "en"
        private set
    private var spokenLang = "en"
    private var tamilAvailable = false

    // Current instruction and what was last spoken.
    private var curCode = "none"
    private var curDist = 0.0
    private var curSince = 0L
    private var pendingSpeak = false
    private var hapticDone = true
    private val lastSpokenAt = HashMap<String, Long>()

    fun start() {
        speech.initialize { applyLanguage() }
    }

    fun release() {
        speech.release()
        haptics.cancel()
    }

    fun configure(enabled: Boolean, lang: String) {
        this.enabled = enabled
        language = if (lang == "ta" || lang == "both") lang else "en"
        prefs.edit().putBoolean("enabled", enabled).putString("lang", language).apply()
        if (!enabled) speech.stop()
        applyLanguage()
        // Say the current instruction again in the new language.
        pendingSpeak = curCode != "none"
        lastSpokenAt.clear()
    }

    private fun applyLanguage() {
        tamilAvailable = speech.isLanguageAvailable(TAMIL)
        // "both": Tamil in the Tamil voice, then English in the English voice (speakParts);
        // without a Tamil voice it falls back to English only.
        spokenLang = if (language != "en" && tamilAvailable && speech.setLanguage(TAMIL)) {
            language
        } else {
            speech.setLanguage(Locale.ENGLISH)
            "en"
        }
        Log.i(TAG, "voice language: chosen $language, speaking $spokenLang (Tamil voice ${if (tamilAvailable) "installed" else "missing"})")
    }

    /** Status for the UI, added to the telemetry. */
    fun statusJson(): JSONObject = JSONObject()
        .put("enabled", enabled)
        .put("lang", language)
        .put("spoken", spokenLang)
        .put("ready", speech.isReady)
        .put("tamil", tamilAvailable)
        .put("screenReader", a11y.isScreenReaderActive.value)

    /** Called with every telemetry update (~10 Hz, main thread). */
    // Dynamic instructions: the object directly ahead (from the safety validator's "ahead"
    // source: a YOLO class or a depth obstacle) and its distance, so instructions say
    // "Chair 2 metres ahead, move right" instead of only "Bear right".
    private var objEn = ""
    private var objTa = ""
    private var objDist = -1.0
    private var objKey = ""          // object + distance step: a change is worth saying again
    private var objSpokenKey = ""

    fun onTelemetry(obj: JSONObject) {
        val guide = obj.optJSONObject("guide") ?: return
        val code = guide.optString("code", "none")
        val dist = guide.optDouble("dist", 0.0)
        val now = SystemClock.elapsedRealtime()
        run {
            val v = obj.optJSONObject("validator")
            val d = v?.optDouble("ahead", -1.0) ?: -1.0
            val names = objectNames(v?.optString("aheadSrc") ?: "")
            if (d > 0 && d <= 3.0 && names != null) {
                objEn = names.first; objTa = names.second; objDist = d
                objKey = "${names.first}:${distStep(d)}"
            } else {
                objEn = ""; objTa = ""; objDist = -1.0; objKey = ""
            }
        }
        // Same instruction, but a new object or it came a step closer: say it again.
        if (code == curCode && code in OBJECT_CODES && objKey.isNotEmpty() && objKey != objSpokenKey &&
            now - (lastSpokenAt[code] ?: 0L) > 2500) pendingSpeak = true
        val textChanged = code == curCode && rounded(dist) != rounded(curDist) && code in DISTANCE_CODES
        if (code != curCode || textChanged) {
            val wasDifferent = code != curCode
            curCode = code
            curDist = dist
            curSince = now
            pendingSpeak = code != "none"
            if (wasDifferent) hapticDone = false
        }
        if (!hapticDone) {
            hapticDone = true
            playHaptic(code)
        }
        if (!enabled || code == "none") return

        val priority = PRIORITY[code] ?: 10
        if (code in HALT) { speakHalt(code, dist, priority, now, guide.optString("text")); return }
        haltSince = 0L   // left the "don't move" group
        // Reminders while a stop-type instruction / camera fault lasts.
        val repeatMs = REPEAT_MS[code]
        val sinceKey = now - (lastSpokenAt[code] ?: 0L)
        if (!pendingSpeak && repeatMs != null && sinceKey >= repeatMs) pendingSpeak = true
        if (!pendingSpeak) return

        // Cooldown for repeating the same instruction (A -> B -> A chatter). A different, more
        // urgent instruction is never held back: it has its own key, and SpeechManager lets a
        // strictly higher priority interrupt speech in progress.
        val cooldown = COOLDOWN_MS[code] ?: 2000L
        if (sinceKey < cooldown) {
            if (repeatMs == null) pendingSpeak = false   // drop: just said it
            return
        }

        val nt = guide.optString("text")
        val ta = dynamicPhrase(code, dist, "ta", nt)
        val en = dynamicPhrase(code, dist, "en", nt)
        if (en.isEmpty()) { pendingSpeak = false; return }
        if (say(code, ta, en, priority, now, 1500)) {
            pendingSpeak = false
            lastSpokenAt[code] = now
            objSpokenKey = objKey
        }
        // Not ok: outranked by speech in progress or engine not ready; offered again next tick
        // while still current.
    }

    // "Don't move" group: Stop / Stop, way is left|right / No clear way / Obstacle directly ahead /
    // camera problem. Spoken once when it starts and repeated every HALT_REPEAT_MS while it
    // lasts; a change inside the group is spoken at once only when it is more urgent or tells
    // the way (left / right) - otherwise it waits for the next repeat. Without this, standing
    // still made the voice alternate "Stop" / "No clear way" every 4-5 s.
    private var haltSince = 0L
    private var haltLastAt = 0L
    private var haltLastPriority = 0
    private var haltLastCode = ""

    private fun speakHalt(code: String, dist: Double, priority: Int, now: Long, nativeText: String) {
        val entering = haltSince == 0L
        if (entering) haltSince = now
        val since = now - haltLastAt
        val due = entering || since >= HALT_REPEAT_MS
        val moreUrgent = priority > haltLastPriority &&
            (since >= HALT_UPGRADE_GAP_MS || code == "sensor_fault")
        val tellsWay = (code == "stop_left" || code == "stop_right") && code != haltLastCode &&
            since >= HALT_UPGRADE_GAP_MS
        if (!(due || (pendingSpeak && (moreUrgent || tellsWay)))) return
        val ta = dynamicPhrase(code, dist, "ta", nativeText)
        val en = dynamicPhrase(code, dist, "en", nativeText)
        if (en.isEmpty()) return
        if (say(code, ta, en, priority, now, 1500)) {
            pendingSpeak = false
            haltLastAt = now
            haltLastPriority = priority
            haltLastCode = code
            lastSpokenAt[code] = now
            objSpokenKey = objKey
        }
    }

    /**
     * The live situation sentence for the on-screen banner (same words as the voice, e.g.
     * "Chair 2 metres ahead, path goes right."), every frame, whether or not it is spoken
     * now. Call after onTelemetry (it uses the object ahead read there). Tamil when the
     * voice is Tamil only, else English. Empty when there is no instruction.
     */
    fun displaySentence(obj: JSONObject): String {
        val g = obj.optJSONObject("guide") ?: return ""
        val code = g.optString("code", "none")
        if (code == "none" || code.isEmpty()) return ""
        return dynamicPhrase(code, g.optDouble("dist", 0.0), if (spokenLang == "ta") "ta" else "en", g.optString("text"))
    }

    /** Speaks [ta] / [en] per the language mode; "both" = Tamil voice, then English voice. */
    private fun say(key: String, ta: String, en: String, priority: Int, now: Long, expiresMs: Long): Boolean {
        val a = Announcement("$key$now", key, if (spokenLang == "ta") ta else en, priority, now + expiresMs)
        val ok = if (spokenLang == "both" && ta.isNotEmpty() && en.isNotEmpty()) {
            speech.speakParts(a, listOf(TAMIL to ta, Locale.ENGLISH to en))
        } else {
            speech.speak(a)
        }
        if (ok) {
            val said = if (spokenLang == "both") "$ta | $en" else a.text
            Log.i(TAG, "SAY,$key,$spokenLang,$said")
            onSay?.invoke(key, said)
        }
        return ok
    }

    /**
     * "What's ahead?" answer (Gemma, English only): lowest priority, so any instruction or
     * STOP interrupts it; spoken in the English voice whatever the language setting.
     */
    fun sayDescription(text: String): Boolean {
        if (!enabled || text.isBlank()) return false
        val now = SystemClock.elapsedRealtime()
        val a = Announcement("d$now", "describe", text, 10, now + 8000)
        val ok = if (spokenLang == "en") speech.speak(a) else speech.speakParts(a, listOf(Locale.ENGLISH to text))
        if (ok) {
            Log.i(TAG, "SAY,describe,en,$text")
            onSay?.invoke("describe", text)
        }
        return ok
    }

    /**
     * Voice assistant response: priority 15, below navigation/hazard alerts (70-100)
     * so emergency and guidance alerts immediately preempt assistant speech.
     */
    fun sayAssistantResponse(text: String): Boolean {
        if (!enabled || text.isBlank()) return false
        val now = SystemClock.elapsedRealtime()
        val a = Announcement("va$now", "assistant", text, 15, now + 10000)
        val ok = if (spokenLang == "en") speech.speak(a) else speech.speakParts(a, listOf(Locale.ENGLISH to text))
        if (ok) {
            Log.i(TAG, "SAY,assistant,en,$text")
            onSay?.invoke("assistant", text)
        }
        return ok
    }

    /** A short status message (start / stop / camera connected), with a confirming vibration. */
    fun notice(en: String, ta: String, haptic: LongArray? = null) {
        haptic?.let { haptics.play(it, 200) }
        if (!enabled) return
        val now = SystemClock.elapsedRealtime()
        say("notice", ta, en, 50, now, 4000)
    }

    fun onNavigationStarted() {
        curCode = "none"; haltSince = 0L
        notice("Navigation started", "வழிகாட்டல் தொடங்கியது", longArrayOf(0, 40, 70, 40))
    }

    fun onNavigationStopped() {
        curCode = "none"; haltSince = 0L
        notice("Navigation stopped", "வழிகாட்டல் நிறுத்தப்பட்டது", longArrayOf(0, 180))
    }

    /**
     * Instruction with the object ahead when one is known (<= 3 m), e.g.
     * "Chair 2 metres ahead, move right." / "STOP! Person right in front."
     * Falls back to the fixed phrase otherwise.
     */
    private fun dynamicPhrase(code: String, dist: Double, lang: String, nativeText: String): String {
        val ta = lang == "ta"
        // Exact situations (user's list):
        //   Chair 2 m ahead, path goes right · Person 2 m ahead, path goes left
        //   Table 1 m ahead, path straight past it · Person very close (within 0.5 m)
        //   Stopped, a table blocks, way on the left · Something at head height
        //   Nothing within 3 m
        if (objEn.isEmpty()) {
            val none = if (ta) "3 மீட்டருக்குள் எதுவும் இல்லை" else "Nothing within 3 metres"
            return when (code) {
                "straight" -> "$none."
                // Fix 2: turns without an object in the same situation style.
                "bear_left", "turn_left" -> if (ta) "$none, வழி இடது பக்கம்." else "$none, path goes left."
                "bear_right", "turn_right" -> if (ta) "$none, வழி வலது பக்கம்." else "$none, path goes right."
                else -> phrase(code, dist, lang, nativeText)
            }
        }
        val o = if (ta) objTa else cap(objEn)
        val n = Math.round(objDist).toInt()
        val veryClose = objDist <= 0.5
        val head = objEn == "something at head height"
        val art = if (objEn.firstOrNull()?.lowercaseChar() in setOf('a', 'e', 'i', 'o', 'u')) "an" else "a"
        val at = if (ta) "$n மீட்டரில் முன்னால் $o" else (if (n == 1) "$o 1 metre ahead" else "$o $n metres ahead")
        if (head && code !in setOf("bear_left", "bear_right", "turn_left", "turn_right"))
            return if (ta) "தலை உயரத்தில் தடை." else "Something at head height."
        return when (code) {
            "bear_right", "turn_right" -> if (ta) "$at, வழி வலது பக்கம்." else "$at, path goes right."
            "bear_left", "turn_left" -> if (ta) "$at, வழி இடது பக்கம்." else "$at, path goes left."
            "straight" -> if (ta) "$at, வழி நேராக அதைக் கடந்து." else "$at, path straight past it."
            // A STOP always says so (phone: "Laptop 1 metre ahead." was spoken for a STOP at
            // 0.6 m - it sounded like information, not a stop). No rounded distance here.
            "obstacle_ahead" -> if (veryClose) (if (ta) "நில்லுங்கள்! மிக அருகில் $o." else "STOP! $o very close.")
                                else (if (ta) "நில்லுங்கள்! நேராக முன்னால் $o." else "STOP! $o directly ahead.")
            "stop" -> if (ta) "நின்றுள்ளீர்கள், முன்னால் $o தடுக்கிறது." else "Stopped, $art ${objEn} blocks."
            "stop_left" -> if (ta) "நின்றுள்ளீர்கள், $o தடுக்கிறது, வழி இடது பக்கம்." else "Stopped, $art ${objEn} blocks, way on the left."
            "stop_right" -> if (ta) "நின்றுள்ளீர்கள், $o தடுக்கிறது, வழி வலது பக்கம்." else "Stopped, $art ${objEn} blocks, way on the right."
            else -> phrase(code, dist, lang, nativeText)
        }
    }

    private fun cap(s: String) = s.replaceFirstChar { it.uppercase() }

    /** The camera was unplugged while running: the stream stops, so no telemetry follows. */
    fun onCameraLost() {
        haptics.play(longArrayOf(0, 150, 80, 150, 80, 150, 400, 150, 80, 150, 80, 150), 255)
        curCode = "sensor_fault"
        if (!enabled) return
        val now = SystemClock.elapsedRealtime()
        say("camera_lost", "நில்லுங்கள்! கேமரா துண்டிக்கப்பட்டது.", "STOP! Camera lost.", 100, now, 3000)
    }

    /** Layer 1 alert vibration (STOP three fast pulses, CAUTION one long), rate-limited. */
    private var lastAlertVibrationMs = 0L
    fun vibrateOnAlert(alert: String) {
        if (alert != "stop" && alert != "caution") return
        val now = SystemClock.elapsedRealtime()
        val minGap = if (alert == "stop") 1000 else 2000
        if (now - lastAlertVibrationMs < minGap) return
        lastAlertVibrationMs = now
        haptics.play(if (alert == "stop") longArrayOf(0, 120, 80, 120, 80, 120) else longArrayOf(0, 400), 255)
    }

    // Direction cues on change: left = short pulses, right = one long pulse (more for a turn).
    private fun playHaptic(code: String) {
        val p = when (code) {
            "bear_left", "turn_left_soon" -> longArrayOf(0, 60, 90, 60)
            "turn_left", "stop_left" -> longArrayOf(0, 60, 90, 60, 90, 60)
            "bear_right", "turn_right_soon" -> longArrayOf(0, 220)
            "turn_right", "stop_right" -> longArrayOf(0, 400)
            else -> return   // stop-type: the alert vibration already covers it
        }
        // Do not overlap a STOP alert pattern that just played.
        if (SystemClock.elapsedRealtime() - lastAlertVibrationMs < 600) return
        haptics.play(p, 200)
    }

    companion object {
        private const val TAG = "HSV2Voice"
        private val TAMIL: Locale = Locale.forLanguageTag("ta-IN")

        // Instructions that can carry the object ahead.
        private val OBJECT_CODES = setOf("straight", "bear_left", "bear_right", "turn_left", "turn_right",
            "obstacle_ahead", "stop", "stop_left", "stop_right")

        private fun distStep(d: Double): Int = if (d < 0.9) 0 else Math.round(d).toInt()

        // COCO class id (YOLO) -> English, Tamil.
        private val COCO = mapOf(
            0 to ("person" to "ஒருவர்"), 1 to ("bicycle" to "சைக்கிள்"), 2 to ("car" to "கார்"),
            3 to ("motorbike" to "இருசக்கர வாகனம்"), 5 to ("bus" to "பேருந்து"), 7 to ("truck" to "லாரி"),
            13 to ("bench" to "பெஞ்ச்"), 15 to ("cat" to "பூனை"), 16 to ("dog" to "நாய்"),
            24 to ("bag" to "பை"), 26 to ("bag" to "பை"), 28 to ("suitcase" to "பெட்டி"),
            39 to ("bottle" to "பாட்டில்"), 56 to ("chair" to "நாற்காலி"), 57 to ("sofa" to "சோபா"),
            58 to ("plant" to "செடி"), 59 to ("bed" to "கட்டில்"), 60 to ("table" to "மேசை"),
            62 to ("TV" to "தொலைக்காட்சி"), 63 to ("laptop" to "மடிக்கணினி"),
            72 to ("fridge" to "குளிர்சாதனப் பெட்டி"),
        )

        /** "det:56" / "obstacle" / "head" / "low" / "trip" -> (English, Tamil), null if none. */
        fun objectNames(src: String): Pair<String, String>? = when {
            src.startsWith("det:") -> COCO[src.removePrefix("det:").toIntOrNull()] ?: ("object" to "பொருள்")
            src == "obstacle" -> "obstacle" to "தடை"
            src == "head" -> "something at head height" to "தலை உயரத்தில் தடை"
            src == "low" -> "low object" to "தாழ்வான பொருள்"
            src == "trip" -> "step or cable" to "படி அல்லது கம்பி"
            else -> null
        }

        private val HALT = setOf("stop", "stop_left", "stop_right", "blocked", "obstacle_ahead", "sensor_fault")
        private const val HALT_REPEAT_MS = 5000L
        // A more urgent / directional change inside the group is spoken at once, but not
        // closer than this to the previous halt message (alerts flickering on and off).
        private const val HALT_UPGRADE_GAP_MS = 1500L

        private val DISTANCE_CODES = setOf("turn_left_soon", "turn_right_soon", "door")

        // Higher = more urgent; strictly higher interrupts speech in progress.
        private val PRIORITY = mapOf(
            "sensor_fault" to 100, "obstacle_ahead" to 95,
            "stop" to 90, "stop_left" to 90, "stop_right" to 90,
            "stairs" to 85, "blocked" to 70,
            "turn_left" to 60, "turn_right" to 60,
            "turn_left_soon" to 50, "turn_right_soon" to 50,
            "door" to 45, "bear_left" to 40, "bear_right" to 40,
            "straight" to 30, "unverified" to 20,
        )
        // Minimum gap before the same instruction is spoken again.
        private val COOLDOWN_MS = mapOf(
            "straight" to 6000L, "unverified" to 6000L, "bear_left" to 3000L, "bear_right" to 3000L,
            "turn_left_soon" to 3000L, "turn_right_soon" to 3000L, "door" to 5000L,
            "turn_left" to 2500L, "turn_right" to 2500L, "blocked" to 5000L, "stairs" to 4000L,
            "stop" to 2500L, "stop_left" to 2500L, "stop_right" to 2500L,
            "obstacle_ahead" to 2000L, "sensor_fault" to 3000L,
        )
        // Repeated while they last (Sailens: sensor problems repeat until resolved).
        private val REPEAT_MS = mapOf(
            "obstacle_ahead" to 3000L, "sensor_fault" to 4000L,
            "stop" to 4000L, "stop_left" to 4000L, "stop_right" to 4000L,
            "blocked" to 6000L, "stairs" to 5000L,
        )

        private fun rounded(d: Double): Int = maxOf(1, Math.round(d).toInt())

        /** Instruction text; must match the meaning of guidance.cpp's makeText. */
        fun phrase(code: String, dist: Double, lang: String, nativeText: String = ""): String {
            if (lang == "both") {
                val ta = phrase(code, dist, "ta", nativeText)
                val en = phrase(code, dist, "en", nativeText)
                return if (ta.isEmpty()) en else "$ta. $en"
            }
            val n = rounded(dist)
            return if (lang == "ta") when (code) {
                "straight" -> "நேராகச் செல்லுங்கள்"
                "bear_left" -> "சற்று இடது பக்கம் செல்லுங்கள்"
                "bear_right" -> "சற்று வலது பக்கம் செல்லுங்கள்"
                "turn_left" -> "இடது பக்கம் திரும்புங்கள்"
                "turn_right" -> "வலது பக்கம் திரும்புங்கள்"
                "turn_left_soon" -> "$n மீட்டரில் இடது பக்கம் திரும்புங்கள்"
                "turn_right_soon" -> "$n மீட்டரில் வலது பக்கம் திரும்புங்கள்"
                "door" -> "$n மீட்டரில் கதவு உள்ளது"
                "stairs" -> "முன்னால் படிக்கட்டுகள் உள்ளன"
                "stop" -> "நில்லுங்கள்"
                "stop_left" -> "நில்லுங்கள், வழி இடது பக்கம்"
                "stop_right" -> "நில்லுங்கள், வழி வலது பக்கம்"
                "blocked" -> "தெளிவான வழி இல்லை"
                "obstacle_ahead" -> "நில்லுங்கள்! நேராக முன்னால் தடை உள்ளது."
                "unverified" -> "காத்திருங்கள், வழியைச் சரிபார்க்கிறேன்"
                "sensor_fault" -> when {
                    "blocked" in nativeText -> "நில்லுங்கள்! கேமரா மறைக்கப்பட்டுள்ளது."
                    "lost" in nativeText -> "நில்லுங்கள்! கேமரா துண்டிக்கப்பட்டது."
                    else -> "நில்லுங்கள்! கேமரா நின்றுவிட்டது."
                }
                else -> ""
            } else when (code) {
                "straight" -> "Go straight"
                "bear_left" -> "Bear left"
                "bear_right" -> "Bear right"
                "turn_left" -> "Turn left"
                "turn_right" -> "Turn right"
                "turn_left_soon" -> "Turn left in $n ${if (n == 1) "metre" else "metres"}"
                "turn_right_soon" -> "Turn right in $n ${if (n == 1) "metre" else "metres"}"
                "door" -> "Door $n ${if (n == 1) "metre" else "metres"} ahead"
                "stairs" -> "Stairs ahead"
                "stop" -> "Stop"
                "stop_left" -> "Stop, way is left"
                "stop_right" -> "Stop, way is right"
                "blocked" -> "No clear way"
                "obstacle_ahead" -> "STOP! Obstacle directly ahead."
                "unverified" -> "Wait, checking the way"
                "sensor_fault" -> when {
                    "blocked" in nativeText -> "STOP! Camera blocked."
                    "lost" in nativeText -> "STOP! Camera lost."
                    else -> "STOP! Camera stalled."
                }
                else -> ""
            }
        }
    }
}
