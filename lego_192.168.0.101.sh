#!/bin/sh
make clean && make && sshpass -p ' ' scp program root@192.168.0.101:/media/card/program && echo Program Transferred...  && sshpass -p ' ' ssh root@192.168.0.101 /media/card/program && echo Done!
