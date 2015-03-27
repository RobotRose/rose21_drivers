#set terminal x11 persist raise



set autoscale        	               # scale axes automatically
unset log                              # remove any log-scaling
unset label                            # remove any previous labels
set xtic auto                          # set xtics automatically
set ytic auto                          # set ytics automatically
set title "PID Rose 2.0 BL drive"
set xlabel "Time"
set ylabel "Error/Effort"
set term wxt 0
plot    "PID_data_drive.dat" using 1:2  title 'Error 1' with linespoints , \
    	"PID_data_drive.dat" using 1:3  title 'Effort 1' with linespoints , \
    	"PID_data_drive.dat" using 1:4  title 'Measured 1' with linespoints , \
    	"PID_data_drive.dat" using 1:5  title 'Set Point 1' with linespoints

set term wxt 1
set title "PID Rose 2.0 BR drive"
plot    "PID_data_drive.dat" using 1:7  title 'Error 2' with linespoints , \
    	"PID_data_drive.dat" using 1:8  title 'Effort 2' with linespoints , \
    	"PID_data_drive.dat" using 1:9  title 'Measured 2' with linespoints , \
    	"PID_data_drive.dat" using 1:10  title 'Set Point 2' with linespoints

set term wxt 2
set title "PID Rose 2.0 FL drive"
plot    "PID_data_drive.dat" using 1:12  title 'Error 3' with linespoints , \
    	"PID_data_drive.dat" using 1:13  title 'Effort 3' with linespoints , \
    	"PID_data_drive.dat" using 1:14  title 'Measured 3' with linespoints , \
    	"PID_data_drive.dat" using 1:15  title 'Set Point 3' with linespoints

set term wxt 3
set title "PID Rose 2.0 FR drive"
plot    "PID_data_drive.dat" using 1:17  title 'Error 4' with linespoints , \
    	"PID_data_drive.dat" using 1:18  title 'Effort 4' with linespoints , \
    	"PID_data_drive.dat" using 1:19  title 'Measured 4' with linespoints , \
    	"PID_data_drive.dat" using 1:20  title 'Set Point 4' with linespoints



