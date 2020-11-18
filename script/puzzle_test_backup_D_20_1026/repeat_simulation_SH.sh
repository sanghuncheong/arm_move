#!/bin/bash

for i in 12 16
do
	for j in {0..34}
	do
		python D_20_1020_vrepMoveit_jaco2.py 0 $j /home/sanghun16/testdata_icra/testdata_$i.txt
	done
done


cd
