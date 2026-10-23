package com.hsv2.hsv2_lite

import android.content.Context
import android.os.SystemClock
import android.util.Log
import org.json.JSONObject
import java.io.BufferedWriter
import java.io.File
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

/**
 * Phase 7 validation: while a test scenario runs, every telemetry update (~10 Hz) and every
 * spoken message is written to a CSV file, so a walk can be scored afterwards on the PC
 * (tools/validate_walks.py). Files go to the app's external files dir, readable with
 * `adb pull /sdcard/Android/data/com.hsv2.hsv2_lite/files/validation`.
 *
 * Rows:  T,<ms>,<telemetry columns...>   one per telemetry update
 *        S,<ms>,<key>,<text>              a spoken message
 *        E,<ms>,<event>                   start / end / camera_lost / note
 * <ms> = milliseconds since the scenario started (monotonic clock).
 */
class ValidationLogger(private val context: Context) {

    private var out: BufferedWriter? = null
    private var t0 = 0L
    private var rowsSinceFlush = 0
    var scenario: String? = null
        private set
    var file: File? = null
        private set

    val active: Boolean get() = out != null

    fun start(scenarioId: String, label: String) {
        stop()
        val dir = File(context.getExternalFilesDir(null), "validation").apply { mkdirs() }
        val stamp = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(Date())
        val f = File(dir, "${stamp}_$scenarioId.csv")
        try {
            out = f.bufferedWriter()
            file = f
            scenario = scenarioId
            t0 = SystemClock.elapsedRealtime()
            out?.apply {
                write("# HearSight validation,$scenarioId,$label,$stamp\n")
                write("# T,ms,fps,status,alert,alertReason,alertDist,verdict,by,mayMove,pathOk,failed," +
                    "vdist,ahead,obst,head,drop,guide,guideSeq,guideDist,health,healthReason,depthValid," +
                    "planOk,planBlocked,planTurn,planLen,planClear,speed,stale,aheadSrc\n")
            }
            event("start")
            Log.i(TAG, "validation start $scenarioId -> ${f.absolutePath}")
        } catch (e: Exception) {
            Log.e(TAG, "cannot open ${f.absolutePath}", e)
            out = null
        }
    }

    fun stop() {
        val w = out ?: return
        event("end")
        try { w.close() } catch (_: Exception) {}
        Log.i(TAG, "validation end $scenario -> ${file?.absolutePath}")
        out = null
        scenario = null
    }

    private fun ms(): Long = SystemClock.elapsedRealtime() - t0

    fun event(name: String) = line("E,${ms()},$name")

    fun said(key: String, text: String) = line("S,${ms()},$key,${text.replace(',', ';')}")

    fun telemetry(o: JSONObject) {
        if (out == null) return
        val v = o.optJSONObject("validator")
        val g = o.optJSONObject("guide")
        val h = o.optJSONObject("health")
        val p = o.optJSONObject("plan")
        val vo = o.optJSONObject("vo")
        fun d(x: Double) = if (x.isNaN()) "" else String.format(Locale.US, "%.2f", x)
        val cols = listOf(
            "T", ms().toString(),
            d(o.optDouble("fps", Double.NaN)), o.optString("status"),
            o.optString("alert"), o.optString("alertReason"), d(o.optDouble("alertDist", Double.NaN)),
            v?.optInt("verdict", -1)?.toString() ?: "", v?.optString("by") ?: "",
            (v?.optBoolean("mayMove") ?: false).toString(), (v?.optBoolean("pathOk") ?: false).toString(),
            v?.optLong("failed", 0)?.toString() ?: "",
            d(v?.optDouble("dist", Double.NaN) ?: Double.NaN), d(v?.optDouble("ahead", Double.NaN) ?: Double.NaN),
            d(v?.optDouble("obst", Double.NaN) ?: Double.NaN), d(v?.optDouble("head", Double.NaN) ?: Double.NaN),
            d(v?.optDouble("drop", Double.NaN) ?: Double.NaN),
            g?.optString("code") ?: "", g?.optInt("seq", 0)?.toString() ?: "",
            d(g?.optDouble("dist", Double.NaN) ?: Double.NaN),
            h?.optInt("level", 0)?.toString() ?: "", h?.optString("reason") ?: "",
            d(h?.optDouble("depthValid", Double.NaN) ?: Double.NaN),
            (p?.optBoolean("ok") ?: false).toString(), (p?.optBoolean("blocked") ?: false).toString(),
            d(p?.optDouble("turn", Double.NaN) ?: Double.NaN), d(p?.optDouble("length", Double.NaN) ?: Double.NaN),
            d(p?.optDouble("clearance", Double.NaN) ?: Double.NaN),
            d(vo?.optDouble("speed", Double.NaN) ?: Double.NaN),
            o.optBoolean("stale", false).toString(),
            v?.optString("aheadSrc") ?: "",
        )
        line(cols.joinToString(","))
        // Written out about once a second, so a walk survives the app closing.
        if (++rowsSinceFlush >= 10) {
            rowsSinceFlush = 0
            try { out?.flush() } catch (_: Exception) {}
        }
    }

    private fun line(s: String) {
        val w = out ?: return
        try { w.write(s); w.write("\n") } catch (e: Exception) { Log.w(TAG, "write failed", e) }
    }

    companion object { private const val TAG = "HSV2Valid" }
}
