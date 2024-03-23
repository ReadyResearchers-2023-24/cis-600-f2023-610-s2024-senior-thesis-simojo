set terminal png size 2000,1200 enhanced font "Roboto Mono,30"
set xlabel "Number of Episodes"
set ylabel "Episode Time (s)"
set key right bottom

set output "../images/plots/plot-episode-duration.png"

plot "deltas.txt" \
using 1:2 with lines \
smooth bezier \
title "Run 1" \
linecolor rgb "#000000" \
linewidth 6
