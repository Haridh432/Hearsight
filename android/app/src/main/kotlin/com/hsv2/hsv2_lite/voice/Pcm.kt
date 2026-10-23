package com.hsv2.hsv2_lite.voice

import java.io.File
import java.io.RandomAccessFile
import java.nio.ByteBuffer
import java.nio.ByteOrder
import kotlin.math.max
import kotlin.math.min
import kotlin.math.roundToInt

/** Mono float PCM in [-1, 1]. */
class Pcm(val samples: FloatArray, val sampleRate: Int = 16000) {
    val seconds: Double get() = samples.size.toDouble() / sampleRate

    /** Linear-interpolation resample; suitable for speech entering 16 kHz ASR. */
    fun resampled(target: Int): Pcm {
        if (target == sampleRate) return this
        val ratio = sampleRate.toDouble() / target
        val n = (samples.size / ratio).toInt()
        val out = FloatArray(n)
        for (i in 0 until n) {
            val pos = i * ratio
            val i0 = pos.toInt()
            val i1 = min(i0 + 1, samples.size - 1)
            val frac = (pos - i0).toFloat()
            out[i] = samples[i0] * (1 - frac) + samples[i1] * frac
        }
        return Pcm(out, target)
    }
}

object Wav {
    /** Reads 16-bit PCM or 32-bit float WAV; multi-channel input is averaged to mono. */
    fun read(file: File): Pcm {
        val bytes = file.readBytes()
        val buf = ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN)
        require(String(bytes, 0, 4) == "RIFF" && String(bytes, 8, 4) == "WAVE") { "Not a WAV file: $file" }
        var pos = 12
        var channels = 1
        var rate = 16000
        var bits = 16
        var format = 1
        while (pos + 8 <= bytes.size) {
            val id = String(bytes, pos, 4)
            val size = buf.getInt(pos + 4)
            val body = pos + 8
            when (id) {
                "fmt " -> {
                    format = buf.getShort(body).toInt()
                    channels = buf.getShort(body + 2).toInt()
                    rate = buf.getInt(body + 4)
                    bits = buf.getShort(body + 14).toInt()
                }
                "data" -> {
                    val frameBytes = channels * bits / 8
                    val frames = min(size, bytes.size - body) / frameBytes
                    val out = FloatArray(frames)
                    for (f in 0 until frames) {
                        var acc = 0f
                        for (c in 0 until channels) {
                            val off = body + f * frameBytes + c * bits / 8
                            acc += when {
                                format == 3 && bits == 32 -> buf.getFloat(off)
                                bits == 16 -> buf.getShort(off) / 32768f
                                bits == 32 -> buf.getInt(off) / 2147483648f
                                else -> error("Unsupported WAV: format=$format bits=$bits")
                            }
                        }
                        out[f] = acc / channels
                    }
                    return Pcm(out, rate)
                }
            }
            pos = body + size + (size and 1)
        }
        error("No data chunk in $file")
    }

    fun write(file: File, pcm: Pcm) {
        file.parentFile?.mkdirs()
        val dataBytes = pcm.samples.size * 2
        RandomAccessFile(file, "rw").use { raf ->
            raf.setLength(0)
            val header = ByteBuffer.allocate(44).order(ByteOrder.LITTLE_ENDIAN)
            header.put("RIFF".toByteArray()).putInt(36 + dataBytes).put("WAVE".toByteArray())
            header.put("fmt ".toByteArray()).putInt(16).putShort(1).putShort(1)
                .putInt(pcm.sampleRate).putInt(pcm.sampleRate * 2).putShort(2).putShort(16)
            header.put("data".toByteArray()).putInt(dataBytes)
            raf.write(header.array())
            val data = ByteBuffer.allocate(dataBytes).order(ByteOrder.LITTLE_ENDIAN)
            for (s in pcm.samples) data.putShort((max(-1f, min(1f, s)) * 32767f).roundToInt().toShort())
            raf.write(data.array())
        }
    }
}
