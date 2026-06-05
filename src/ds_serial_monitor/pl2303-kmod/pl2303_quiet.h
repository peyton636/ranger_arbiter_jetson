/* Disable dynamic debug for out-of-tree Jetson build (symvers mismatch). */
#undef dev_dbg
#define dev_dbg(dev, fmt, ...) do { } while (0)
