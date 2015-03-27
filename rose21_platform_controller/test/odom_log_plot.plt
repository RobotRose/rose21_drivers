#set terminal x11 persist raise
set autoscale        	               # scale axes automatically
unset log                              # remove any log-scaling
unset label                            # remove any previous labels
set xtic auto                          # set xtics automatically
set ytic auto                          # set ytics automatically
set title "Odometry Rose 2.0"
set xlabel "Time"
set ylabel "rad | m/s"
plot    "odom_log.dat" using 1:2 title 'Rotation (rad) FL' with linespoints , \
    	"odom_log.dat" using 1:3 title 'Rotation (rad) BL' with linespoints , \
    	"odom_log.dat" using 1:4 title 'Rotation (rad) FR' with linespoints , \
    	"odom_log.dat" using 1:5 title 'Rotation (rad) BR' with linespoints , \
    	"odom_log.dat" using 1:6 title 'Speed (m/s) FL' with linespoints , \
    	"odom_log.dat" using 1:7 title 'Speed (m/s) BL' with linespoints , \
    	"odom_log.dat" using 1:8 title 'Speed (m/s) FR' with linespoints , \
    	"odom_log.dat" using 1:9 title 'Speed (m/s) BR' with linespoints , \
    	"odom_log.dat" using 1:10 title 'Pos x (m)' with linespoints , \
    	"odom_log.dat" using 1:11 title 'Pos y (m)' with linespoints , \
    	"odom_log.dat" using 1:12 title 'Angle (rad)' with linespoints , \
    	"odom_log.dat" using 1:13 title 'Vel x (m/s)' with linespoints , \
    	"odom_log.dat" using 1:14 title 'Vel y (m/s)' with linespoints , \
    	"odom_log.dat" using 1:15 title 'Angle vel (rad/s)' with linespoints 