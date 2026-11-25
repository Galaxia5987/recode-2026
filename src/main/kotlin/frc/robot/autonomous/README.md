# Choreo Paths

Paths are drawn in [Choreo](https://choreo.autos) and saved to `src/main/deploy/choreo`.

## Generated code

KSP reads every `.traj` file and generates, in `frc.robot.autonomous.paths`:

- An object per trajectory, named after it in Pascal case (`fuel to hub` → `FuelToHub`).
- `ChoreoMarkers`, an abstract class with one abstract function per marker name across
  all trajectories, converted to camel case (`Intake Down` → `intakeDown()`).

```kotlin
// Generated from StartToFuel.traj and FuelToHub.traj
object StartToFuel : ChoreoPath("StartToFuel", Markers)
object FuelToHub : ChoreoPath("FuelToHub", Markers)

abstract class ChoreoMarkers : MarkerBindings {
    protected abstract fun Coroutine.intakeDown()
    protected abstract fun Coroutine.shoot()
}
```

The code is regenerated on every build, so adding or renaming a marker in Choreo becomes
a compile error until its binding is written. Names that don't form a valid identifier,
or that map to the same identifier, fail the build.

## Binding markers

Every marker is bound once, globally, in the `Markers` object:

```kotlin
object Markers : ChoreoMarkers() {
    override fun Coroutine.intakeDown() = Intake.open().fork()
    override fun Coroutine.shoot() = +Shooter.shoot()
}
```

A marker with the same name runs the same binding in every path. Bindings run in the
path command's coroutine. Awaiting a command (`+command`) blocks: the drive stops and the
path's timer pauses until it finishes. To run a command alongside the path, fork it.
Forked commands are canceled when the path ends, so fork them from the routine instead
to keep them running longer.

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
