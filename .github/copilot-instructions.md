This file contains instructions for reviewing Kotlin-based FRC code using the custom CommandV3 architecture, WPILib, Phoenix 6, and AdvantageKit. Apply these rules rigorously when evaluating pull requests or new code.

## 1. General Style and Formatting

* **Comments:** Insert only meaningful comments that explain *why* something is done, not *what* is done. Remove redundant or obvious comments.
* **Tone:** Maintain a professional, concise tone. Do not use emojis in your review feedback or in the generated code.

## 2. CommandV3 Architecture and Structure

This project uses a custom coroutine-based command framework (CommandV3).

* **Execution Blocking:** Most commands inside mechanisms should be written to block (using a `while` loop or `waitUntil` with a yield) until they meet their setpoint. This ensures sequential composition works correctly by default. Callers can use `.fork()` if they wish to run the command asynchronously.
* **One-Shot Commands:** Use one-shot commands (no loop, finishes immediately) only when semantically appropriate, such as toggling a solenoid or setting a boolean flag.
* **Yielding:** Any continuous action or waiting loop inside a command *must* call `yield()` at the end of each iteration to prevent locking the main thread. Using `waitUntil` or `Trigger.waitUntil` is good, as it includes `yield()`.
* **waitUntil:** The `waitUntil` function cannot be used safely for `Trigger` objects. `Trigger` objects should use the `Trigger.waitUntil()` extension function.
* **Command Naming:** Every command must end with `.named("...")`. Enforce naming conventions that reflect the file, subsystem, and action (e.g., `.named("Elevator/SetToScoringPosition")`).
* **Trigger Locality:** Prefer triggers nested inside commands over globally defined triggers whenever possible. This encapsulates behavior and prevents unpredictable state changes.
* **Composition:** Ensure proper use of `+` for sequential execution and `+[]` for parallel execution within a `command` or mechanism block.

## 3. Mechanism and Subsystem Design

* **Unified State Naming:** Mechanisms must use a single, unified naming convention for target states and completion checks. Always use `setpoint` for the target variable and `atSetpoint` for the boolean completion check. Do not use variations like `isFinished`, `target`, `goalReached`, or `atTarget`.
* **State Machine Validation:** Scrutinize state machines and logic flows. Ensure there are no dead-ends, that default states are safe, and that transitions handle edge cases (e.g., sensor failures or unexpected inputs).

## 4. Hardware and Motor Configuration (Phoenix 6 / UniversalTalonFX)

* **Current Limits:** Every motor configuration *must* include explicitly defined current limits (`StatorCurrentLimitEnable`, `SupplyCurrentLimitEnable`, and their respective values). Flag any motor configuration missing these.
* **Soft Limits:** Mechanisms with physical boundaries (e.g., elevators, arms, wrists) must implement soft limits to prevent self-destruction.
* **Control Requests:** Validate that the appropriate Phoenix 6 `ControlRequest` is used for the specific use case.
* Verify whether `VoltageOut`, `PositionVoltage`, or `MotionMagicTorqueCurrentFOC` is being used and if it correctly aligns with the mechanism's physics and tuning.
* Ensure that feedforward is applied correctly within the control request where applicable.


* **UniversalTalonFX Usage:** Ensure initialization leverages the existing `UniversalTalonFX` wrapper properly, utilizing `gearRatio` and `linearSystemWheelDiameter` arguments correctly to prevent manual unit conversion math in the subsystem logic.

## 5. Logging and AdvantageKit

* **Comprehensive Logging:** More logging is always preferred. Ensure all critical mechanism states, sensor inputs, control requests, setpoints, and calculated errors are logged.
* **Inputs Architecture:** Verify that IO layers and `AutoLogInputs` are utilized correctly for hardware abstraction.
* **Annotations and Extensions:** Check for the proper use of `@LoggedOutput`, custom `.log()` extension functions, and `AutoLogOutputManager` registrations.
* **Log Paths:** Ensure logging paths are clean, organized, and properly namespaced (e.g., `Subsystems/Elevator/setpoint`).
