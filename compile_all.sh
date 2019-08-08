#!/bin/bash
cd FreeRTOS/Demo/RISC-V_Galois_P1/
for prog in main_blinky main_full main_iic main_uart main_udp main_tcp
do
	make clean
        export PROG=$prog
        make
        if [ $? -eq 0 ]
        then
            echo $PROG OK
        else
            echo $PROG failed
            exit 1
        fi
done
make clean
cd -
