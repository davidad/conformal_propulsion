all: exp ni

exp: main.c Makefile
	gcc -I/usr/local/natinst/nidaqmxbase/include -lnidaqmxbase -lconfig -g -O3 main.c -o exp

ni: ni.c Makefile
	gcc -I/usr/local/natinst/nidaqmxbase/include -lnidaqmxbase -g -O3 ni.c -o ni
