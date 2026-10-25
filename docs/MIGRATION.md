# Source migration record

The consolidation is copy-based.  No original source, trajectory, or handoff
file was moved or overwritten.

## Current planning sources

Files under `laptop/pipeline` came from the 2026-09-03 output directory.  The
required `execute_left_taught_with_auto_start.py` base and the read-only teach
recorder came from the 2026-09-01 output directory.  The hard-coded dependency
on that dated directory was replaced by a project-local dependency.

Planning inputs were separated from generated runs:

- raw teaching input: `data/raw`
- current taught CDR and metadata: `data/taught`
- verified orientation reference: `data/references`
- generated results: `runs/<timestamp>`

## Geometry invalidation

The original PlanningScene helper contains a fixed table center of
`[0.48, 0.00, -0.18]` in `torso_link`. After the column raise, the onsite
operator confirmed on 2026-09-07 that the existing compensated RViz command
remains correct and requested that it remain canonical. The value is therefore
pinned in both `active_geometry.json` and
`geometry/column_15cm_existing_tf_operator_confirmed.json`.

## Historical artifacts

Full trajectories and visual micro-adjustment artifacts made before the column
raise were moved only inside the new project to
`archive/stale_after_column_raise_15cm`.  Their original copies remain in the
source handoff directory.
