#!/bin/bash
# ============================================================================
#  e310_capture.sh  --  record the E310 boot console on every USB serial port
#
#      bash ~/Downloads/e310_capture.sh
#
#  The E310 is powered by the same USB cable that carries the console, so the
#  serial port only exists after power-on and the boot text is printed in the
#  first seconds. This script waits for the port, starts recording at once,
#  sends "?" + Enter after 15 s and prints what the board sent.
#  Logs: ~/e310_ttyUSB0.log, ~/e310_ttyUSB1.log, ...
# ============================================================================
sudo -v || exit 1                                   # ask for the password up front
pkill -f "putty" 2>/dev/null                        # PuTTY must not hold the port

echo
echo ">>> 1) UNPLUG the E310 USB cable, then press Enter"
read -r
if ls /dev/ttyUSB* >/dev/null 2>&1; then
  echo "    note: still present (another USB serial device?):"; ls /dev/ttyUSB*
fi
echo ">>> 2) PLUG IN the E310 now ... (waiting for /dev/ttyUSB*)"
until ls /dev/ttyUSB* >/dev/null 2>&1; do sleep 0.05; done
sleep 0.3

rm -f ~/e310_ttyUSB*.log
for p in /dev/ttyUSB*; do
  n=$(basename "$p")
  sudo stty -F "$p" 115200 cs8 -cstopb -parenb raw -echo -ixon -ixoff -crtscts
  sudo cat "$p" > ~/e310_"$n".log &
  echo "    recording $p -> ~/e310_$n.log"
done

echo ">>> 3) recording boot for 15 s ..."; sleep 15
for p in /dev/ttyUSB*; do printf '\r?\r' | sudo tee "$p" >/dev/null; done
echo ">>> 4) sent '?' + Enter, waiting 3 s ..."; sleep 3
sudo pkill -f "cat /dev/ttyUSB"

echo; echo "===== ports"; ls -l /dev/ttyUSB*
echo; echo "===== kernel"; sudo dmesg | grep -iE "ttyUSB|ftdi|cp210|ch34" | tail -6
for f in ~/e310_ttyUSB*.log; do
  echo; echo "===== $f  ($(stat -c %s "$f") bytes)"; cat -v "$f" | head -80
done
echo; echo ">>> copy everything above and paste it into the chat"
