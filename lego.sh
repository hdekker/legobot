#!/bin/sh
make clean && make && sshpass -p ' ' scp program root@192.168.3.197:/media/card/program && sshpass -p ' ' ssh root@192.168.3.197 /media/card/program && echo Done!
