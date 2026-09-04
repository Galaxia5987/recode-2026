package frc.robot.lib.extensions

import java.util.Collections
import java.util.WeakHashMap
import kotlin.properties.ReadOnlyProperty
import kotlin.reflect.KProperty

/**
 * A delegate to process a variable once per loop execution.
 *
 * Prevents "getter hell", where an expensive getter calculation is repeatedly
 * called within a single loop iteration (including nested getter calls).
 *
 * ### Example
 *
 * ```kotlin
 * val someProperty by periodic {
 *     // Expensive calculation...
 * }
 * ```
 */
class PeriodicDelegate<T>(private val calculation: () -> T) :
    ReadOnlyProperty<Any?, T> {

    private var isDirty: Boolean = true
    private var cachedValue: T? = null

    fun invalidate() {
        isDirty = true
    }

    override fun getValue(thisRef: Any?, property: KProperty<*>): T {
        if (isDirty) {
            cachedValue = calculation()
            isDirty = false
        }

        @Suppress("UNCHECKED_CAST")
        return cachedValue as T
    }
}

object CacheManager {
    private val delegates: MutableSet<PeriodicDelegate<*>> =
        Collections.newSetFromMap(WeakHashMap())

    fun register(delegate: PeriodicDelegate<*>) {
        delegates += delegate
    }

    fun invalidateAll() {
        delegates.forEach {
            it.invalidate()
        }
    }
}

fun <T> periodic(calculation: () -> T): PeriodicDelegate<T> =
    PeriodicDelegate(calculation).also {
        CacheManager.register(it)
    }
