for file in `ls *.pdf`; do export test=`basename "$file" .pdf`; convert -density 100 $file "$test"_%04d.png; done
mv Swarm_Robot_Control_-_10_Robot*.png ../10
mv Swarm_Robot_Control_-_100_Robot*.png ../100
mv Swarm_Robot_Control_-_1000_Robot*.png ../1000
mv Swarm_Robot_Control_-_Single_Robot*.png ../1
mv Swarm_Robot_Control_-_Unknown_Number_of_Robots*.png ../unknown
