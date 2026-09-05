# Lost-line search axle-speed experiment

Start commit: `2a7a5d5611928205137df858b448086f34685821`.
User observation: the apparent rotation centre remains too far forward during
lost-line search. Normal on-line cornering is outside this change.

Search commands now use `DriveBase_SetWheelCps` with this wheel order:

| Direction | M1 front left | M2 rear left | M3 front right | M4 rear right |
|---|---:|---:|---:|---:|
| Left | -3600 | -2160 | +3600 | +2160 |
| Right | +3600 | +2160 | -3600 | -2160 |

Units are encoder counts/second. Each axle still has zero mean longitudinal
target. There is no new forward/reverse bias between the left and right wheels.
The four motor polarity mappings and DriveBase feedback are unchanged.

`LINE_TRACKING_SEARCH_REAR_PERCENT` in `line_tracking.h` defaults to 60.
Compile with 100 to reproduce the previous equal-axle search targets. Supported
values are 50 through 100; at 50 the rear target magnitude remains 1800 CPS.
Zero is deliberately not supported: DriveBase zero-speed output coasts instead
of locking the rear axle. All search entries and restarts share one helper.

This changes wheel travel demand, not measured torque or a specified ground
pivot. Tyre slip, load and surface determine whether the apparent rotation
centre moves, and by how much. No geometric claim about a rear-axle pivot is
made. The 60-percent value is an experiment, not a calibrated improvement.

Normal on-line commands, capture speeds, search-direction logic, reversal
braking, widening sweep durations and the 8-second watchdog are unchanged.
The host runner exercises the default and 100-percent configurations, checking
all four signed targets in both search directions and after reversals/retries,
as well as restoration of equal-axle commands on capture and normal tracking.

Build and host tests cannot validate chassis motion. The integration task
should compare 60 and 100 percent on the same surface, load and battery level,
observing the front and rear of the chassis from above and checking both search
directions, capture success and unintended translation. If the effect is absent
or worse, retain 100 rather than interpreting encoder counts as chassis yaw.

This branch starts at the requested older integration baseline. Later changes
such as buzzer GPIO initialization are not included in its standalone binary.
Merge the source commit into the current integration branch and build there;
integration owns any eventual flashing and physical tests. This task does not
edit PROJECT_STATE.md or access hardware.
