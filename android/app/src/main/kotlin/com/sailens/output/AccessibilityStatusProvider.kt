// Copied unchanged from Sailens Android (Apache-2.0), see third_party/sailens.
package com.sailens.output

import android.content.Context
import android.view.accessibility.AccessibilityManager
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow

/**
 * 观察系统里是否有屏幕阅读器（TalkBack 一类）正在工作。
 *
 * 这决定了导航提示走哪条播报通道。两条通道**只能开一条**：本应用自带的 TTS，或者交给
 * 屏幕阅读器播报。以前两条同时开着，同一句提示会被念两遍，而且互不同步、彼此重叠——
 * 对目标用户来说这不是小瑕疵，是提示直接不可用。
 *
 * 交给屏幕阅读器的那条通道还有个附带好处：语速、音量、语言全部继承用户在 TalkBack 里的
 * 既有设置，而这些设置对重度用户是高度个性化的。
 *
 * 判据用 `isEnabled && isTouchExplorationEnabled`：触摸浏览是屏幕阅读器的特征功能，
 * 而单看 `isEnabled` 会把开关控制、放大手势等非朗读类无障碍服务也算进来。
 */
public class AccessibilityStatusProvider(context: Context) {
    private val manager = context.applicationContext
        .getSystemService(Context.ACCESSIBILITY_SERVICE) as? AccessibilityManager

    private val _isScreenReaderActive = MutableStateFlow(computeActive())
    public val isScreenReaderActive: StateFlow<Boolean> = _isScreenReaderActive.asStateFlow()

    init {
        // 应用级单例，生命周期与进程一致，因此不需要反注册。
        // 用户可能在使用过程中开关 TalkBack，播报通道要跟着切。
        manager?.addTouchExplorationStateChangeListener { refresh() }
        manager?.addAccessibilityStateChangeListener { refresh() }
    }

    private fun refresh() {
        _isScreenReaderActive.value = computeActive()
    }

    private fun computeActive(): Boolean {
        val accessibilityManager = manager ?: return false
        return accessibilityManager.isEnabled && accessibilityManager.isTouchExplorationEnabled
    }
}
