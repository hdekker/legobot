#!/bin/sh
make clean && make && sshpass -p ' ' scp program root@192.168.3.198:/media/card/program && sshpass -p ' ' ssh root@192.168.3.198 /media/card/program && echo Done!
