#!/bin/bash


for i in 16
do
	for j in {17..17}
	do
		python D_00_16_sh_test.py 0 $j /home/sanghun16/testdata_icra/testdata_$i.txt
	done
done


cd
