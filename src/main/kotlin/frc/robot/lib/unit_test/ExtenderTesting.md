# Subsystem simulation testing

This example uses `Extender`, but the same pattern can be used for any
subsystem built with `UniversalTalonFX`.

## A basic test

```kotlin
class ExtenderTests : Tests() {
    @Test
    fun `extender reaches its target`() {
        val target = 10.m

        SubsystemSimRuntime.apply {
            addCommands(Extender.setPosition(target))
            runFor(10.sec)
        }

        assertTrue(
            getInputs(PORT).distance.isNear(target, 0.2.m),
            "actual distance: ${getInputs(PORT).distance}"
        )
    }
}
```

The test has three steps:

1. Create the subsystem command.
2. Run the command in simulation.
3. Read the simulated motor inputs and check the result.

## Scheduling commands

Use `addCommands()` instead of scheduling the command directly:

```kotlin
SubsystemSimRuntime.addCommands(
    Extender.setPosition(10.m)
)
```

This schedules the command and registers any required subsystem that
implements `SimulationResettable`.

The command must declare the subsystem as a requirement for automatic
registration to work. Commands created with `SubsystemBase.runOnce`,
`run`, and similar helpers normally do this automatically.

Multiple commands can be added together:

```kotlin
SubsystemSimRuntime.addCommands(commandA, commandB)
```

## Advancing simulation

Run a fixed amount of simulated time with:

```kotlin
SubsystemSimRuntime.runFor(5.sec)
```

The runtime advances time in 20 ms steps and runs the WPILib command
scheduler after every step.

Use `runUntil()` when the test should finish as soon as a condition becomes
true:

```kotlin
val reachedTarget =
    SubsystemSimRuntime.runUntil(5.sec) {
        Extender.atSetpoint.asBoolean
    }

assertTrue(reachedTarget, "Extender did not reach its target in time")
```

`runUntil()` returns `false` if the timeout expires first.

## Reading simulated motor inputs

`UniversalTalonFX` registers its simulated motor using its CAN ID and CAN bus.
Read it with:

```kotlin
val inputs = getInputs(PORT)

val distance = inputs.distance
val position = inputs.position
val velocity = inputs.velocity
val voltage = inputs.voltage
val current = inputs.statorCurrent
```

The default CAN bus is `"rio"`. Specify another bus when necessary:

```kotlin
val inputs = getInputs(port = 11, canbus = "canivore")
```

## Cleaning state between tests

Test classes should inherit from `Tests`:

```kotlin
class ExtenderTests : Tests()
```

After every test, the inherited cleanup:

- cancels scheduled commands;
- clears composed commands;
- resets registered `SimulationResettable` subsystems;
- resets all registered simulated motor inputs.

A subsystem only needs to implement `SimulationResettable` when it owns
mutable state that could affect the next test:

```kotlin
object ExampleSubsystem : SubsystemBase(), SimulationResettable {
    private var setpoint = 0.m

    override fun resetSimulationState() {
        setpoint = 0.m
    }
}
```

Motor inputs are reset automatically, so they do not need to be reset again
inside `resetSimulationState()`.

## Complete condition-based example

```kotlin
class ExtenderTests : Tests() {
    @Test
    fun `open command reaches its setpoint`() {
        SubsystemSimRuntime.addCommands(Extender.open())

        val completed =
            SubsystemSimRuntime.runUntil(5.sec) {
                Extender.atSetpoint.asBoolean
            }

        assertTrue(completed, "Extender did not open within 5 seconds")
        assertTrue(
            getInputs(PORT).distance.isNear(
                ExtenderPositions.OPEN.distance,
                EXTENDER_SETPOINT_TOLERANCE
            )
        )
    }
}
```

