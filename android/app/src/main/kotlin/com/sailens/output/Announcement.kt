// Copied unchanged from Sailens Android (Apache-2.0), see third_party/sailens.
package com.sailens.output

/**
 * Something to say, with everything the output layer needs to decide how to say it -- and
 * nothing about why it is being said.
 *
 * The text arrives already resolved. sailens-output does not know what a scene event is, what a
 * navigation hazard is, or which product asked; deciding what to announce and in what words is
 * product policy and belongs above this module (architecture.md §6.6). What stays here is the
 * mechanism: expiry, priority preemption, audio focus, the engine's readiness.
 */
public data class Announcement(
    /** Stable id for this utterance, used to correlate TTS callbacks. */
    val id: String,
    /** Diagnostic label. Appears in logs only, never spoken. */
    val key: String,
    val text: String,
    /**
     * Something newer interrupts this only by being strictly higher. An equal or lower one that
     * arrives while this is still being spoken is refused, and the caller decides whether to offer
     * it again. The scale is the caller's; this module only compares.
     */
    val priority: Int,
    /**
     * Deadline on the `SystemClock.elapsedRealtime()` clock. Guidance speech describes a moving
     * world, so an announcement that missed its moment is worse than silence -- the person cannot
     * see that it is stale. Monotonic rather than wall-clock: a clock set backwards must not make
     * stale speech look fresh, nor fresh speech look stale.
     */
    val expiresAtMs: Long,
) {
    public fun isExpired(nowMs: Long): Boolean = nowMs > expiresAtMs
}
