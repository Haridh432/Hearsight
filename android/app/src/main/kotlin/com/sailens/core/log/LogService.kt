// Copied unchanged from Sailens Android (Apache-2.0), see third_party/sailens.
package com.sailens.core.log

/**
 * 日志服务接口
 */
public interface LogService {
    public fun debug(tag: String, message: String, data: Map<String, Any>?  = null)
    public fun info(tag: String, message: String, data: Map<String, Any>? = null)
    public fun warning(tag: String, message: String, data: Map<String, Any>? = null, throwable: Throwable? = null)
    public fun error(tag: String, message: String, throwable: Throwable?  = null)
}
