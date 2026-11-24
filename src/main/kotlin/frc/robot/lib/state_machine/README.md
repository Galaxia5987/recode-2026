# State Machine Builder DSL

Usage Example:

```kotlin
enum class ShooterState {
    IDLE,
    SPIN_UP,
    SHOOT,
    COOLDOWN;

    companion object {
        val stateMachine = buildStateMachine<ShooterState>("Shooter State Machine") {
            IDLE {
                shooter.stop()
            }.initial()

            SPIN_UP(shooter.runSpinUpCommand())

            SHOOT {
                shooter.feedNote()
            }

            COOLDOWN {
                shooter.idleFlywheels()

            }

            IDLE on shootTrigger switchTo SPIN_UP

            SPIN_UP on isAtTargetSpeed switchTo SHOOT

            SHOOT.onComplete switchTo COOLDOWN

            COOLDOWN.onComplete switchTo IDLE

            allOf<ShooterState>() on { !shootTrigger.asBoolean } switchTo IDLE
        }
    }
}
```

## Enter and exit callbacks

```kotlin
SHOOT onEnter { leds.flash() }

// Runs on every transition out of SHOOT and when the whole machine is
// cancelled, so it is the right place to stop motors the state started.
SHOOT onExit { feeder.stop() }
```

`onStateChange { }` can be called more than once; every callback runs.

## Forcing a transition

`stateMachine.forceTransition(IDLE)` leaves the current state on the next loop,
running its exit callbacks and then the target's enter callbacks. It does
nothing while the machine is not scheduled.
