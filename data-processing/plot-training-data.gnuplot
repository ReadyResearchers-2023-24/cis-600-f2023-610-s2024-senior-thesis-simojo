set terminal png size 2000,1200 enhanced font "Roboto Mono,30"
set xlabel "Number of Episodes"
set ylabel "Average Reward"
set key right bottom

#https://colordesigner.io/gradient-generator
#e66b6b
#ddb863
#a6d35d
#57c860
#53bca4
#5083af

set output "images/plots/1.png"
plot "../SimonJonesArtifact/temp/clover_train_metadata/clover_train-2024-03-21T00:04:46.769488.txt" \
skip 2 \
using 1:3 with linespoints \
smooth bezier \
title "1" \
linecolor rgb "#e66b6b" \
linewidth 6

set output "images/plots/2.png"
plot "../SimonJonesArtifact/temp/clover_train_metadata/clover_train-2024-03-20T21:42:50.323611.txt" \
skip 2 \
using 1:3 with linespoints \
smooth bezier \
title "2" \
linecolor rgb "#ddb863" \
linewidth 6

set output "images/plots/3.png"
plot "../SimonJonesArtifact/temp/clover_train_metadata/clover_train-2024-03-20T01:20:48.933112.txt" \
skip 2 \
using 1:3 with linespoints \
smooth bezier \
title "3" \
linecolor rgb "#a6d35d" \
linewidth 6

set yr [-17000:0]
set output "images/plots/4.png"
plot "../SimonJonesArtifact/temp/clover_train_metadata/clover_train-2024-03-17T22:25:51.450693.txt" \
skip 2 \
using 1:3 with linespoints \
smooth bezier \
title "4" \
linecolor rgb "#57c860" \
linewidth 6
set yr [*:*]

set output "images/plots/5.png"
plot "../SimonJonesArtifact/temp/clover_train_metadata/clover_train-2024-03-16T03:58:01.007264.txt" \
skip 2 \
using 1:3 with linespoints \
smooth bezier \
title "5" \
linecolor rgb "#53bca4" \
linewidth 6

set output "images/plots/6.png"
plot "../SimonJonesArtifact/temp/clover_train_metadata/clover_train-2024-03-15T06:20:20.154119.txt" \
skip 2 \
using 1:3 with linespoints \
smooth bezier \
title "6" \
linecolor rgb "#5083af" \
linewidth 6

set output "images/plots/multiplot.png"
plot \
     "../SimonJonesArtifact/temp/clover_train_metadata/clover_train-2024-03-21T00:04:46.769488.txt" \
     skip 2 \
     using 1:3 with lines \
     smooth bezier \
     title "1" \
     linecolor rgb "#e66b6b" \
     linewidth 6, \
     "../SimonJonesArtifact/temp/clover_train_metadata/clover_train-2024-03-20T21:42:50.323611.txt" \
     skip 2 \
     using 1:3 with lines \
     smooth bezier \
     title "2" \
     linecolor rgb "#ddb863" \
     linewidth 6, \
     "../SimonJonesArtifact/temp/clover_train_metadata/clover_train-2024-03-20T01:20:48.933112.txt" \
     skip 2 \
     using 1:3 with lines \
     smooth bezier \
     title "3" \
     linecolor rgb "#a6d35d" \
     linewidth 6, \
     "../SimonJonesArtifact/temp/clover_train_metadata/clover_train-2024-03-17T22:25:51.450693.txt" \
     skip 2 \
     using 1:3 with lines \
     smooth bezier \
     title "4" \
     linecolor rgb "#57c860" \
     linewidth 6, \
     "../SimonJonesArtifact/temp/clover_train_metadata/clover_train-2024-03-16T03:58:01.007264.txt" \
     skip 2 \
     using 1:3 with lines \
     smooth bezier \
     title "5" \
     linecolor rgb "#53bca4" \
     linewidth 6, \
     "../SimonJonesArtifact/temp/clover_train_metadata/clover_train-2024-03-15T06:20:20.154119.txt" \
     skip 2 \
     using 1:3 with lines \
     smooth bezier \
     title "6" \
     linecolor rgb "#5083af" \
     linewidth 6
