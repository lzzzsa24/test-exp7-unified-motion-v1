# This branch builds the new, standalone four-sensor application.
# Kept as the repository's standard build entrypoint.
param([switch]$LineTrackingLiftTest)
if ($LineTrackingLiftTest) { throw 'Legacy lift-test mode is absent on feature/simple-four-line.' }
& (Join-Path $PSScriptRoot 'build_simple_line.ps1')
