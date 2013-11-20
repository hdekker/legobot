#!/bin/sh
make clean && make && sshpass -p ' ' scp program root@192.168.0.105:/media/card/program && echo Done!
