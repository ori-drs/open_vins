#!/usr/bin/env bash
set -euo pipefail

runs=(
  "floor_1_2025-05-05_run_1"   # good
  "floor_1_2025-07-07_run_1"   # good, require zero init bias
  "floor_1_2025-12-02_run_1"   # good

  "floor_2_2025-05-05_run_1"   # good
  "floor_2_2025-10-28_run_1"   # good, require zero init bias
  "floor_2_2025-10-28_run_2"   # good
  "floor_2_2025-12-02_run_1"   # good
  "floor_2_2025-12-03_run_1"   # good

  "floor_3_2025-05-19_run_1"   # good, can easily fail
  "floor_3_2025-12-02_run_1"   # good

  "floor_4_2025-05-19_run_1"   # good, require zero init bias
  "floor_4_2025-12-02_run_1"   # good, require zero init bias

  "floor_5_2025-12-02_run_1"   # good, can easily fail
)

for run in "${runs[@]}"; do
  evo_traj tum \
    "/media/yuhao/bluessd/hilti/BuchsIT/ov_output/${run}.txt" \
    --ref "/media/yuhao/bluessd/hilti/BuchsIT/groundtruth/${run}.txt" \
    -p -a --t_max_diff 0.02
done
