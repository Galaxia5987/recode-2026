# A Guide to WPILib CommandV3 in Kotlin

CommandV3 is meant to be an easier way to write with the Command based framework.
At first, it might seem a bit unintuitive, but it gets better.

---

## Creating Commands

Commands are now made using coroutines.

Coroutines are like lambdas, but have a special function called `yield`. This special function gives but control to the scheduler, until the next loop.
This makes it possible to write loops inside function that seem like it will block the execution, but it doesn't.

We can look at the following simple example of creating a simple `run` command. This is equivilant to `Commands.run` from CommandV2.
```kotlin
// CommandV3
fun command(): Command = noRequirements {
    while(true){
        // Do work
        !this
    }
}.named("SomeCommand")
```

> `!this` is the yield function 

This command will run this loop at 50Hz. This is the same as writing this in CommandV2:
```kotlin
// CommandV2
fun command(): Command = Commands.run {
    // Do work
}
```

We can also make commands that only runs once.
```kotlin
// CommandV3
fun command(): Command = noRequirements {
    // Do work
}
```

This command will run once and exit.
This is the same as writing this in CommandV2:
```kotlin
fun command(): Command = Commands.runOnce {
    // Do work
}
```

## Defining Commands inside a Mechanism

Commands can also have mechanism requirements:

### Instant Actions (`runOnce` Equivalent)

If a command simply needs to fire an action and finish immediately, you write the logic without any loops. Once the block completes, the command is done.

```kotlin
object Intake : Mechanism() {
    
    fun open() = this {
        deploySolenoid.set(true)
    }.named("Deploy Intake")
    
}

```

### Continuous Actions (`run` Equivalent)

If a command needs to continuously run. use an infinite loop and call `!this` at the end of each iteration.

```kotlin
object Intake : Mechanism() {
    
    fun runIntake() = this {
        while (true) {
            motor.set(1.0)
            !this 
        }
    }.named("Run Intake Continuously")
    
}

```

### Conditional Actions (Replacing `isFinished`)

To run an action until a specific condition is met, use a standard `while` loop. Any cleanup code (previously in `end()`) goes directly after the loop.

```kotlin
object Intake : Mechanism() {
    
    fun intakeUntilNote() = this {
        while (!stopButtonPressed.get()) {
            motor.set(1.0)
            !this
        }
        motor.set(0.0) 
    }.named("Intake Until Note")
    
}

```

> **Mandatory Naming:** You will notice `.named("...")` appended to every command. CommandV3 enforces strict naming. If you forget to provide a name, the builder will not compile into a valid `Command` object.

---

## Composing Commands

CommandV3 eliminates the need for `SequentialCommandGroup` and `.andThen()`. Because commands are just functions running on coroutines, you can sequence them by simply awaiting their completion.

### Sequential Execution

To run commands one after another, place them on sequential lines preceded by `+`. Scheduling an inner command automatically handles mechanism requirements safely.

```kotlin
fun scoreRoutine() = noRequirements {
    +Intake.open()
    +Shooter.setVelocity(30.0)
    +Intake.close()
}.named("Score Routine")

```

### Parallel Execution

To run a few commands in parallel we can use a collection literal.

```kotlin
fun prepareToScore(intake: IntakeMechanism, shooter: ShooterMechanism, elevator: ElevatorMechanism) = noRequirements {
    +[intake.deploy(), shooter.spinUp(), elevator.moveToScoringPosition()]
    
    +intake.feedShooter()
}.named("Prepare and Score")

```