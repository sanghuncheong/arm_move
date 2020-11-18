#!/bin/bash

for i in 16
do
	for j in {0..34}
	do
		python D_00_sh_test.py 0 $j /home/sanghun16/testdata_icra/testdata_$i.txt
	done
done

for i in 16
do
	for j in {0..34}
	do
		python D_01_sh_test.py 0 $j /home/sanghun16/testdata_icra/testdata_$i.txt
	done
done

cd
