#!/bin/bash
# DISABLED 16:32: both FIFO-boosting (15:31) and taskset-pinning (16:19) the Soapy plugin worker threads were followed
# 10-17 min later by a TX feed freeze (sw_count stuck) and an endless re-anchor storm. Leave scheduling alone.
echo "$(date -u +%H:%M:%S) thread scheduling left untouched (no-op)"
exit 0
