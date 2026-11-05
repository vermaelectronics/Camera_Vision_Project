#!/usr/bin/env bash
# Step 10: what passed, what failed, and what it means for the design.
# ./10_report.sh --paths   also lists every failing timing path (1-2 min)
cd "$(dirname "$0")/../../.."
./Automation/Linux/build_report.sh "$@"
