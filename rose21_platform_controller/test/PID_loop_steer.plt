#set terminal x11 persist raise
set autoscale        	               # scale axes automatically
unset log                              # remove any log-scaling
unset label                            # remove any previous labels
set xtic auto                          # set xtics automatically
set ytic auto                          # set ytics automatically
set title "PID Rose 2.0 steer"
set xlabel "Time"
set ylabel "Error/Effort"
plot    "PID_data_steer.dat" using 1:2 title 'Error 1' with linespoints , \
    	"PID_data_steer.dat" using 1:3 title 'Effort 1' with linespoints , \
    	"PID_data_steer.dat" using 1:4 title 'Error 2' with linespoints , \
    	"PID_data_steer.dat" using 1:5 title 'Effort 2' with linespoints , \
    	"PID_data_steer.dat" using 1:6 title 'Error 3' with linespoints , \
    	"PID_data_steer.dat" using 1:7 title 'Effort 3' with linespoints , \
    	"PID_data_steer.dat" using 1:8 title 'Error 4' with linespoints , \
    	"PID_data_steer.dat" using 1:9 title 'Effort 4' with linespoints 
