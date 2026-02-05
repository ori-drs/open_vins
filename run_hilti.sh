#!/usr/bin/env bash
set -euo pipefail

BASE_BAG_DIR="/media/yuhao/bluessd/hilti/BuchsIT"
BASE_OUT_DIR="/media/yuhao/bluessd/hilti/BuchsIT/ov_output"

# Floors before floor_5: floor_1 .. floor_4 from your list
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

mkdir -p "$BASE_OUT_DIR"

for run in "${runs[@]}"; do
  # run format: floor_<ID>_<YYYY-MM-DD>_run_<N>
  # Examples:
  #   floor_2_2025-10-28_run_1
  #   floor_EG_2025-12-02_run_2
  IFS="_" read -r floor_prefix floor_id date run_label run_num <<< "$run"

  floor="${floor_prefix}_${floor_id}"     # e.g. floor_2
  run_id="${run_label}_${run_num}"        # e.g. run_1

  bag_path="${BASE_BAG_DIR}/${floor}/${date}/${run_id}/rosbag"
  out_path="${BASE_OUT_DIR}/${floor}_${date}_${run_id}.txt"

  if [[ ! -d "$bag_path" ]]; then
    echo "WARNING: bag path not found, skipping: $bag_path" >&2
    continue
  fi

  echo "============================================================"
  echo "Running: $run"
  echo "  bag:  $bag_path"
  echo "  out:  $out_path"
  echo "============================================================"

  ros2 launch ov_msckf subscribe.launch.py \
    config:=insta_hilti \
    bag:="$bag_path" \
    save_total_state:=true \
    filepath_odom:="$out_path" \
    bag_rate:=1.0

  # ros2 launch ov_msckf serial.launch.py \
  #   config:=insta_hilti \
  #   bag:="$bag_path" \
  #   save_total_state:=true \
  #   filepath_odom:="$out_path"
done
