#!/bin/bash

BINARY="./build/ramulator2"
LOG_DIR="simple_run_log"
mkdir -p "$LOG_DIR"

CFG_DBX_NDP="configuration/dbxdimm_ndp_test.yaml"
CFG_ASYNCDIMM="configuration/asyncdimm_nma_test.yaml"

echo "=== Simple Run: Host=470.lbm, NDP=8K_GEMV ==="
echo ""

# Run DBX-DIMM and AsyncDIMM in parallel
echo "[1/2] DBX-DIMM ..."
$BINARY -f $CFG_DBX_NDP > "$LOG_DIR/dbx.log" 2>&1 &
pid_dbx=$!

echo "[2/2] AsyncDIMM ..."
$BINARY -f $CFG_ASYNCDIMM > "$LOG_DIR/async.log" 2>&1 &
pid_async=$!

# Wait for both
wait $pid_dbx
rc_dbx=$?
wait $pid_async
rc_async=$?

echo ""
echo "=== Results ==="
echo ""

# DBX-DIMM
if [ $rc_dbx -eq 0 ]; then
    cyc_dbx=$(grep -oP 'Total simulation cycles: \K\d+' "$LOG_DIR/dbx.log" | head -1)
    bw_dbx=$(grep -oP 'DB<->DRAM\s+:\s+\K[0-9.]+' "$LOG_DIR/dbx.log" | head -1)
    echo "DBX-DIMM:  cycles=$cyc_dbx  BW=${bw_dbx} GB/s"
else
    echo "DBX-DIMM:  FAILED (rc=$rc_dbx, see $LOG_DIR/dbx.log)"
fi

# AsyncDIMM
if [ $rc_async -eq 0 ]; then
    cyc_async=$(grep -oP 'Total simulation cycles: \K\d+' "$LOG_DIR/async.log" | head -1)
    bw_async=$(grep -oP 'Total Bandwidth\s+:\s+\K[0-9.]+' "$LOG_DIR/async.log" | head -1)
    echo "AsyncDIMM: cycles=$cyc_async  BW=${bw_async} GB/s"
else
    echo "AsyncDIMM: FAILED (rc=$rc_async, see $LOG_DIR/async.log)"
fi

echo ""
echo "Logs: $LOG_DIR/dbx.log, $LOG_DIR/async.log"
