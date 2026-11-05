# boot.py - Executed on boot (including power-cycle boot)
import gc
import esp

esp.osdebug(None)
gc.collect()