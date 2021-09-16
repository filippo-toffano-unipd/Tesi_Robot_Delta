set term png size 1080, 1080
set palette rgb 33,13,10
set output "q.png"
set multiplot layout 2, 2
set grid

set size 0.5, 0.5
set origin 0, 0.5
set title 'Position'
set xlabel 'time [s] / total time [s]'
set ylabel 'motor angle [rad]'
plot\
    'data/data_q1.dat' w l tit 'q1',\
    'data/data_q2.dat' w l tit 'q2',\
    'data/data_q3.dat' w l tit 'q3'

set size 0.5, 0.5
set origin 0.5, 0.5
set title 'Velocity'
set xlabel 'time [s] / total time [s]'
set ylabel 'motor velocity [rad/s]'
plot\
    'data/data_q1I.dat' w l tit 'q1',\
    'data/data_q2I.dat' w l tit 'q2',\
    'data/data_q3I.dat' w l tit 'q3'

set size 0.5, 0.5
set origin 0.0, 0.0
set title 'Acceleration'
set xlabel 'time [s] / total time [s]'
set ylabel 'motor acceleration [rad/s^2]'
plot\
    'data/data_q1II.dat' w l tit 'q1',\
    'data/data_q2II.dat' w l tit 'q2',\
    'data/data_q3II.dat' w l tit 'q3'

set size 0.5, 0.5
set origin 0.5, 0.0
set title 'Torque'
set xlabel 'time [s] / total time [s]'
set ylabel 'motor torque [N*m]'
plot\
    'data/data_torque1.dat' w l tit 'q1',\
    'data/data_torque2.dat' w l tit 'q2',\
    'data/data_torque3.dat' w l tit 'q3'


unset multiplot

set term png size 1080, 1080
set output "pg.png"
set multiplot layout 2, 2

set size 0.5, 0.5
set origin 0, 0.5
set title 'Position'
set xlabel 'time [s] / total time [s]'
set ylabel 'position [m]'
plot\
    'data/data_x.dat' w l tit 'x',\
    'data/data_y.dat' w l tit 'y',\
    'data/data_z.dat' w l tit 'z'

set size 0.5, 0.5
set origin 0.0, 0.0
set title 'Velocity'
set xlabel 'time [s] / total time [s]'
set ylabel 'velocity [m/s]'
plot\
    'data/data_xI.dat' w l tit 'x',\
    'data/data_yI.dat' w l tit 'y',\
    'data/data_zI.dat' w l tit 'z'

set size 0.5, 0.5
set origin 0.5, 0.0
set title 'Acceleration'
set xlabel 'time [s] / total time [s]'
set ylabel 'acceleration [m/s^2]'
plot\
    'data/data_xII.dat' w l tit 'x',\
    'data/data_yII.dat' w l tit 'y',\
    'data/data_zII.dat' w l tit 'z'

set size 0.5, 0.5
set origin 0.5, 0.5
set xrange [-0.598:0.598]
set yrange [-0.598:0.598]
set zrange [-0.84:-0.59]
set title 'Position 3D'
set ticslevel 0
set xlabel 'x [m]'
set ylabel 'y [m]'
set zlabel 'z [m]'
splot\
    'data/data_xyz.dat' w l tit 'position' palette,\