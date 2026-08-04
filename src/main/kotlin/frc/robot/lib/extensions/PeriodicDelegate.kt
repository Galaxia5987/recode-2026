package frc.robot.lib.extensions

import kotlin.properties.ReadOnlyProperty
import kotlin.reflect.KProperty

/*
 * A delegate to process a certain variable once per loop.
 * This is to avoid getter hell, where an expensive getter is called multiple times per loop, often even nested inside other getters.
 *
 * USAGE:
 * val someProperty by periodic {
 *   // Expensive calculation...
 * }
 */

class PeriodicDelegate<T> (
    private val calculation: () -> T
) : ReadOnlyProperty<Any?, T>{

    private var isDirty: Boolean = true
    private var cachedValue: T? = null

    fun invalidate() {
        isDirty = true
    }

    override fun getValue(thisRef: Any?, property: KProperty<*>): T {
        if(isDirty || cachedValue == null){
            cachedValue = calculation()
            isDirty = false
        }

        @Suppress("UNCHECKED_CAST")
        return cachedValue as T
    }

}

object CacheManager {
    private val delegates = mutableListOf<PeriodicDelegate<*>>()

    fun register(delegate: PeriodicDelegate<*>) {
        delegates.add(delegate)
    }

    fun invalidateAll() {
        delegates.forEach {
            it.invalidate()
        }
    }
}

fun <T> periodic(calculation: ()->T) : PeriodicDelegate<T> {
    return PeriodicDelegate(calculation).also {
        CacheManager.register(it)
    }
}
