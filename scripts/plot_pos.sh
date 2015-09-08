#!/bin/bash

cat res.log | grep End_pos | gawk '{print $6" "1-$7}' > pos.log

cat >pos.plot <<EOF
plot [0:1][0:1] "pos.log" u 1:2
pause -1
EOF

gnuplot pos.plot