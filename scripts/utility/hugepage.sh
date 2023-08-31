echo $1 > /proc/sys/vm/nr_hugepages
cat /proc/meminfo | grep Huge
