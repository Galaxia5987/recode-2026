The Choreo wrappers expose the public operations of the pinned Choreo
`AutoFactory`, `AutoRoutine`, and `AutoTrajectory` APIs using Commands v3.
They do not require running the Commands v2 scheduler.

```kotlin
fun exampleAuto(): Command {
    val routine = RobotContainer.autoFactory.newRoutine("Example")
    val path = routine.trajectory("line")
    routine.active().onTrue(
        path.resetOdometry().andThen(path.cmd()).named("ResetAndFollow")
    )
    // path.atTime("intake").onTrue(Intake.intake())
    // path.done().onTrue(nextPath.cmd())
    return routine.cmd()
}
```

Schedule the returned command through the v3 scheduler or add it to the auto
chooser. Creating a command does not execute it. Following a trajectory does
not implicitly reset odometry, matching upstream Choreo's behavior.

Factory operations include all trajectory and reset overloads, trajectory
transforms, optional trajectory logging, `bind`, `cache`, `warmupCmd`, and
`newRoutine`. Routines expose trajectory creation, polling, observation,
aggregate triggers, reset, kill, and command creation with an optional finish
condition. Trajectories expose poses, raw trajectory data, transforms,
completion/time/pose/translation triggers, chaining, and event collection.

`ChoreoTrigger` accepts v3 commands and supports boolean combinations, rising
and falling edges, while bindings, and toggles. Its bindings belong to the
routine; ending or canceling the routine cancels their command children.
`kill()` is permanent for that routine and leaves unrelated commands alone.

`spawnCmd()` queues a trajectory under its routine so it can outlive the
one-shot spawn command while still being canceled when the routine ends.
Manual `poll()` is also supported; call `reset()` when stopping a manually
polled routine. `loop()` exposes its event loop for custom callbacks.

As in upstream Choreo, factory event bindings apply to subsequently created
routine trajectories, not standalone `trajectoryCmd` calls. Trajectory
triggers and spawning require a routine trajectory. Reset and following
honor alliance flipping; an unknown alliance cannot start a routine when
flipping is enabled. Use the optional `scheduler` constructor argument for
an independent v3 scheduler in tests.
