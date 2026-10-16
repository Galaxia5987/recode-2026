This module rebuilds the published Choreo `2027.0.0-alpha-3` Java sources
against the robot's WPILib dependencies. The published binary references
`org.wpilib.util.Alert`, which moved to `org.wpilib.driverstation.Alert` in
WPILib alpha-6 and otherwise crashes during trajectory loading.

`prepareChoreoSources` extracts the pinned sources artifact and changes that
package reference before compilation. Generated sources stay in `build/`;
upstream copyright notices are preserved. No changes are made to trajectory
formats, path following, or alliance flipping.

The unused v2 `AutoChooser` and `AutoChooserOpMode` classes are excluded because
they depend on WPILib's removed Tunable API. The robot already uses
`LoggedDashboardChooser` with Commands v3.

The root project excludes the original Choreo Java binary and depends on this
module instead. Commands v2 remains necessary for Choreo and is adapted to v3
by `ChoreoAutoFactory`.

When a compatible upstream release becomes available, remove this module,
its include in `settings.gradle`, and the root dependency/exclusion, then
update the Choreo vendordep to the compatible release.
