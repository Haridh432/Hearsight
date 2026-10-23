package com.hsv2.hsv2_lite

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.SystemClock
import org.json.JSONArray
import org.json.JSONObject
import kotlin.math.atan2
import kotlin.math.sqrt

/**
 * Phone IMU used in place of the D455 IMU (which stalls the camera's USB
 * link). Assumes the phone is rigidly mounted with the camera: phone upright
 * (portrait), rear camera facing the same way as the D455.
 *
 * Phone axes: x right, y up along the screen, z out of the screen (towards
 * the user). Gravity reads about (0, +9.8, 0) when upright and level.
 *   pitch > 0 : looking above the horizon (rear camera tilted up)
 *   roll  > 0 : rolled clockwise as seen by the user holding the phone
 * zero() stores the current angles as the mount offset.
 */
class PhoneImu(context: Context) : SensorEventListener {

    private val sensorManager =
        context.getSystemService(Context.SENSOR_SERVICE) as SensorManager

    private val gyroSensor = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
    private val accelSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
    private val gravitySensor = sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY)

    private val lock = Any()
    private var running = false

    private val gyro = FloatArray(3)
    private val accel = FloatArray(3)
    private val gravity = FloatArray(3)
    private var haveGravity = false

    private var pitchOffsetDeg = 0f
    private var rollOffsetDeg = 0f

    // Sample-rate measurement over ~1 s windows.
    private var gyroCount = 0
    private var accelCount = 0
    private var gyroHz = 0f
    private var accelHz = 0f
    private var windowStartMs = 0L

    fun start() {
        synchronized(lock) {
            if (running) return
            running = gyroSensor != null && (gravitySensor != null || accelSensor != null)
            windowStartMs = SystemClock.elapsedRealtime()
            gyroCount = 0
            accelCount = 0
        }
        // 5 ms = 200 Hz requested; actual rate depends on the phone.
        gyroSensor?.let { sensorManager.registerListener(this, it, 5000) }
        accelSensor?.let { sensorManager.registerListener(this, it, 5000) }
        gravitySensor?.let { sensorManager.registerListener(this, it, 10000) }
    }

    fun stop() {
        sensorManager.unregisterListener(this)
        synchronized(lock) { running = false }
    }

    /** Use the current orientation as level (mount offset). */
    fun zero() {
        synchronized(lock) {
            val (p, r) = rawAngles()
            pitchOffsetDeg = p
            rollOffsetDeg = r
        }
    }

    override fun onSensorChanged(event: SensorEvent) {
        synchronized(lock) {
            when (event.sensor.type) {
                Sensor.TYPE_GYROSCOPE -> {
                    event.values.copyInto(gyro, 0, 0, 3)
                    gyroCount++
                }
                Sensor.TYPE_ACCELEROMETER -> {
                    event.values.copyInto(accel, 0, 0, 3)
                    accelCount++
                    // Fallback gravity if the phone has no fused gravity sensor.
                    if (gravitySensor == null) {
                        for (i in 0..2) {
                            gravity[i] = if (haveGravity) gravity[i] + 0.1f * (accel[i] - gravity[i]) else accel[i]
                        }
                        haveGravity = true
                    }
                }
                Sensor.TYPE_GRAVITY -> {
                    event.values.copyInto(gravity, 0, 0, 3)
                    haveGravity = true
                }
            }

            val now = SystemClock.elapsedRealtime()
            if (now - windowStartMs >= 1000) {
                val sec = (now - windowStartMs) / 1000f
                gyroHz = gyroCount / sec
                accelHz = accelCount / sec
                gyroCount = 0
                accelCount = 0
                windowStartMs = now
            }
        }
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {}

    // Angles from the gravity ("up") vector; caller holds lock.
    private fun rawAngles(): Pair<Float, Float> {
        if (!haveGravity) return Pair(0f, 0f)
        val (x, y, z) = Triple(gravity[0], gravity[1], gravity[2])
        val pitch = Math.toDegrees(atan2(-z, sqrt(x * x + y * y)).toDouble()).toFloat()
        // Sign verified on the SM8850 phone: clockwise roll (seen from behind) -> positive.
        val roll = Math.toDegrees(atan2(x, y).toDouble()).toFloat()
        return Pair(pitch, roll)
    }

    private fun wrap180(deg: Float): Float {
        var d = deg
        while (d > 180f) d -= 360f
        while (d < -180f) d += 360f
        return d
    }

    /** Same shape as the native "imu" telemetry object, plus "source". */
    fun toJson(): JSONObject = synchronized(lock) {
        val (p, r) = rawAngles()
        JSONObject()
            .put("running", running && haveGravity)
            .put("source", "phone")
            .put("pitch", wrap180(p - pitchOffsetDeg).toDouble())
            .put("roll", wrap180(r - rollOffsetDeg).toDouble())
            .put("gyro", JSONArray(gyro.map { it.toDouble() }))
            .put("accel", JSONArray(accel.map { it.toDouble() }))
            .put("gyroHz", gyroHz.toDouble())
            .put("accelHz", accelHz.toDouble())
    }
}
