# Choreo Paths

Paths are drawn in [Choreo](https://choreo.autos) and saved to `src/main/deploy/choreo`.

## Generated code

For every `.traj` file, KSP generates an abstract class in `frc.robot.autonomous.paths`
named after the trajectory with a `Path` suffix. It has one abstract function per event
marker name, converted to camel case (`Intake Down` → `intakeDown()`). Each function runs
in the routine's `Coroutine`.

```kotlin
// Generated from StartToFuel.traj
abstract class StartToFuelPath : ChoreoPath("StartToFuel") {
    protected abstract fun Coroutine.intakeDown()
    protected abstract fun Coroutine.shoot()
}
```

The code is regenerated on every build, so renaming or adding a marker in Choreo
becomes a compile error until the matching binding is written.

## Binding events

Implement the generated class as an object:

```kotlin
object StartToFuel : StartToFuelPath() {
    override fun Coroutine.intakeDown() = Intake.open().fork()
    override fun Coroutine.shoot() = +Shooter.shoot()
}
```

Bindings run in the path command's coroutine. Awaiting a command (`+command`) blocks:
the drive stops and the path's timer pauses until it finishes. To run a command
alongside the path, fork it. Forked commands are canceled when the path ends, so fork
them from the routine instead to keep them running longer.

Event bindings must not require the drive, since the path holds it.

## Following

`StartToFuel.follow(mirrored = false)` is a command that follows the path, flipped for
the red alliance. `mirrored` mirrors it across the field's long axis, to run a path
drawn for one side of the field on the other side. It uses `Drive.followTrajectory`,
which uses each sample's velocity as feedforward plus PID on x, y and heading.

## Routines

An auto routine is the whole autonomous: a name and a coroutine that awaits or forks
paths and any other commands. Declare routines in `Autos`; each one is added to the
auto chooser when it is created.

```kotlin
object Autos {
    val twoPiece =
        autoRoutine("Two Piece") {
            +StartToFuel.resetOdometry()
            Intake.open().fork()
            +StartToFuel.follow()
            +Shooter.shoot()
            +FuelToHub.follow(mirrored = true)
        }
}
```

Start a routine by awaiting `resetOdometry` on its first path, with the same `mirrored`
value it is followed with.
