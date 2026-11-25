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

Bindings are blocking: awaiting a command (`+command`) stops the drive and pauses the
path until it finishes. To run a command alongside the path, fork it. Forked commands
keep running until the routine ends, even across paths.

## Routines

`autoRoutine` creates a command that requires the drive. Inside it, paths are followed
one after another, and any other command can be awaited or forked between them:

```kotlin
fun twoPiece(mirrored: Boolean) =
    autoRoutine("Two Piece", mirrored) {
        StartToFuel.follow()
        +Shooter.shoot()
        FuelToHub.follow()
    }
```

A routine:

1. Resets odometry to the start of its first path.
2. Mirrors every path across the field's long axis if `mirrored` is set, and flips it
   for the red alliance.
3. Follows the samples with `Drive.followTrajectory`, which uses each sample's velocity as
   feedforward plus PID on x, y and heading.
4. Runs each marker's binding when its time is reached. Markers at the very end of a path
   run after the drive stops.

Event bindings must not require the drive, since the routine holds it.

Every path object is collected into `ChoreoPaths.all` and added to the auto chooser as a
single-path routine, twice: as is, and mirrored. Register multi-path routines in
`RobotContainer` yourself.
