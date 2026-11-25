# Choreo Paths

Paths are drawn in [Choreo](https://choreo.autos) and saved to `src/main/deploy/choreo`.

## Generated code

For every `.traj` file, KSP generates an abstract class in `frc.robot.autonomous.paths`
named after the trajectory with a `Path` suffix. It has one abstract function per event
marker name, converted to camel case (`Intake Down` → `intakeDown()`).

```kotlin
// Generated from StartToFuel.traj
abstract class StartToFuelPath : ChoreoPath("StartToFuel") {
    abstract fun intakeDown(): Command
    abstract fun shoot(): Command
}
```

The code is regenerated on every build, so renaming or adding a marker in Choreo
becomes a compile error until the matching command is bound.

## Binding events

Implement the generated class as an object:

```kotlin
object StartToFuel : StartToFuelPath() {
    override fun intakeDown() = Intake.open()
    override fun shoot() = Shooter.shoot()
}
```

Every such object is collected into `ChoreoPaths.all` and added to the auto chooser
twice: as is, and mirrored.

## Following

`StartToFuel.follow(mirrored = false)` returns a command that:

1. Mirrors the path across the field's long axis if `mirrored` is set, and flips it for
   the red alliance.
2. Resets odometry to the path's start pose.
3. Follows the samples with `Drive.followTrajectory`, which uses each sample's velocity as
   feedforward plus PID on x, y and heading.
4. Forks each marker's command when its time is reached.

Event commands are children of the follow command, so they are canceled when the path
ends and must not require the drive.

Paths can be composed like any other command:

```kotlin
fun twoPiece() = command {
    +StartToFuel.follow()
    +Shooter.shoot()
}.named("Two Piece")
```
