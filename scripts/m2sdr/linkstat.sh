#!/bin/bash
# linkstat.sh <console log>: sums the per-second link metrics rows of the gNB console.
grep -a "^ *[0-9]* *4[0-9a-f]\{3\} |" "$1" | awk -F'|' '{split($2,d," "); split($3,u," "); dok+=d[5]; dnok+=d[6]; uok+=u[6]; unok+=u[7]; snr+=u[1]; n++}
  END{ if(n) printf "rows=%d DL ok/nok=%d/%d (%.1f%%) UL ok/nok=%d/%d (%.1f%%) pusch_snr=%.1f dB\n", n, dok, dnok, 100*dnok/(dok+dnok+1e-9), uok, unok, 100*unok/(uok+unok+1e-9), snr/n }'
