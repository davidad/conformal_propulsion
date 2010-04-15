#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <libconfig.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <time.h>
#include "NIDAQmxBase.h"

config_t *config;
int ps1;
int ps2;
FILE* logfd;
FILE* graphfd;
long long start_time;
TaskHandle nidaq;

void load_config() {
	config = malloc(sizeof(config_t));
	config_init(config);
	config_read_file(config, "current.cfg");
}

void init_log() {
	char logfile[255];
	char graphfile[255];
	char date[128];
	time_t t = time(NULL);
	strftime(date, 128, "%F_%T", localtime(&t));
	const char* logsuffix;
	const char* graphsuffix;
	config_lookup_string(config, "logfile_suffix", &logsuffix);
	snprintf(logfile, 255, "%s%s", date, logsuffix);
	logfd = fopen(logfile, "w");
	struct timeval tv;
	gettimeofday(&tv, NULL);
	start_time = tv.tv_sec*1000000 + tv.tv_usec;
	fprintf(logfd, "Starting at: %s.%06d\n\n", date, tv.tv_usec);
	config_lookup_string(config, "graphfile_suffix", &graphsuffix);
	snprintf(graphfile, 255, "%s%s", date, graphsuffix);
	graphfd = fopen(graphfile, "w");
	config_setting_t* plotter = config_lookup(config, "plotter");
	if(plotter) {
		char buf[255];
		double xmin, xmax, ymin, ymax;
		const char *xlabel, *ylabel, *size;
		config_setting_lookup_float(plotter, "xmin", &xmin);
		config_setting_lookup_float(plotter, "xmax", &xmax);
		config_setting_lookup_float(plotter, "ymin", &ymin);
		config_setting_lookup_float(plotter, "ymax", &ymax);
		config_setting_lookup_string(plotter, "x", &xlabel);
		config_setting_lookup_string(plotter, "y", &ylabel);
		config_setting_lookup_string(plotter, "size", &size);
		sprintf(buf, "tail -f %s | graph -T X -X '%s' -Y '%s' -x %lf %lf -y %lf %lf --bitmap-size '%s' &", graphfile, xlabel, ylabel, xmin, xmax, ymin, ymax, size);
		system(buf);
	}
}


double read_nidaq(TaskHandle* tp, const config_setting_t* dcfg) {
	const char* chan;
	double max;
	double sampleRate;
	int samplesPerBurst;

	float64 data[1000];
	int32 pointsRead;
	char source[] = "OnboardClock";

	config_setting_lookup_string(dcfg, "channel", &chan);
	config_setting_lookup_float(dcfg, "voltage_swing", &max);
	config_setting_lookup_float(dcfg, "sample_rate", &sampleRate);
	config_setting_lookup_int(dcfg, "samples_per_burst", &samplesPerBurst);

	DAQmxBaseCreateTask("", tp);
	DAQmxBaseCreateAIVoltageChan(*tp, chan, "", DAQmx_Val_NRSE, -max, max, DAQmx_Val_Volts, NULL);
	DAQmxBaseCfgSampClkTiming(*tp, source, sampleRate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps, samplesPerBurst);
	DAQmxBaseStartTask(*tp);
	DAQmxBaseReadAnalogF64(*tp, -1, 10.0, 0, data, samplesPerBurst, &pointsRead, NULL);
	DAQmxBaseStopTask(*tp);
	DAQmxBaseClearTask(*tp);
	
	double result = 0.0;
	int i;
	for(i=0;i<pointsRead;i++) {
		result += data[i];
	}
	return (result)/((float)samplesPerBurst);
}

void init_serial(int* fdp, const config_setting_t* tcfg) {
	const char* device;
	int baud_rate;
	speed_t baud;
	int parity_odd, enable_parity;
	struct termios tio;

	config_setting_lookup_string(tcfg, "device", &device);
	config_setting_lookup_bool(tcfg, "parity_odd", &parity_odd);
	config_setting_lookup_bool(tcfg, "enable_parity", &enable_parity);
	config_setting_lookup_int(tcfg, "baud_rate", &baud_rate);

	printf("Setting up serial port %s:\n", device);

	int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
	if(fd<0) {
		perror(device);
		exit(-1);
	}
	tcgetattr(fd,&tio);

	printf("\topened successfully\n");

	switch(baud_rate) {
		case 300:
		baud = B300; break;
		case 600:
		baud = B600; break;
		case 1200:
		baud = B1200; break;
		case 2400:
		baud = B2400; break;
		case 4800:
		baud = B4800; break;
		default:
		baud = B9600; baud_rate=9600; break;
	}
	printf("\tset baud rate to %d\n", baud_rate);

	tio.c_cflag = baud | CSTOPB | CLOCAL | CREAD;
	if(enable_parity) {
		tio.c_cflag |= PARENB;
		tio.c_cflag |= CS7;
		printf("\tparity enabled\n");
		if(parity_odd) {
			tio.c_cflag |= PARODD;
			printf("\t\t(odd)\n");
		} else {
			printf("\t\t\(even)\n");
		}
	} else {
		tio.c_cflag |= CS8;
		printf("\tparity disabled\n");
	}
	tio.c_iflag = IGNPAR | IGNBRK;
	if(enable_parity) {
		tio.c_iflag |= INPCK;
	}
	tio.c_oflag = 0;
	tio.c_lflag = 0;
	tio.c_cc[VMIN] = 1;
	tio.c_cc[VTIME] = 0;
	
	tcsetattr(fd, TCSANOW, &tio);
	tcflush(fd, TCIFLUSH);
	tcflush(fd, TCOFLUSH);
	int serial = TIOCM_DTR;
	ioctl(fd,TIOCMSET,&serial);
	*fdp = fd;
}

void plog(char* format, ...) {
	char msg[2048];
	va_list args;
	va_start(args, format);
	vsprintf(msg, format, args);
	struct timeval tv;
	gettimeofday(&tv, NULL);
	long long timeu = tv.tv_sec*1000000 + tv.tv_usec;
	fprintf(logfd,"%llu usec\n%s\n", (timeu-start_time), msg);
	fflush(logfd);
	va_end(args);
}

void pgraph(double psuv, double psua, double psuw, double niv) {
	const char *x, *y;
	config_lookup_string(config, "plotter/x", &x);
	config_lookup_string(config, "plotter/y", &y);
	if(!strcmp(x,"PSU Volts")) {
		fprintf(graphfd, "%lf\t", psuv);
	} else if(!strcmp(x,"PSU Amps")) {
		fprintf(graphfd, "%lf\t", psua);
	} else if(!strcmp(x,"PSU Watts")) {
		fprintf(graphfd, "%lf\t", psuw);
	} else if(!strcmp(x,"NI Volts")) {
		fprintf(graphfd, "%lf\t", niv);
	}
	if(!strcmp(y,"PSU Volts")) {
		fprintf(graphfd, "%lf\n", psuv);
	} else if(!strcmp(y,"PSU Amps")) {
		fprintf(graphfd, "%lf\n", psua);
	} else if(!strcmp(y,"PSU Watts")) {
		fprintf(graphfd, "%lf\n", psuw);
	} else if(!strcmp(y,"NI Volts")) {
		fprintf(graphfd, "%lf\n", niv);
	}
	fflush(graphfd);
}

void ps1_wait_for_dtr() {
	int serial = 0;
	do {
		ioctl(ps1, TIOCMGET, &serial);
		//printf("serial: %d\n", serial);
	} while(!(serial & 0x100));
}

void ps1_send(char* format, ...) {
	char buf[100];
	va_list args;
	va_start(args, format);
	vsprintf(buf, format, args);
	ps1_wait_for_dtr();
	write(ps1, buf, strlen(buf));
	plog("SENT TO PS1:\n===============\n%s===============\n", buf);
	va_end(args);
}

void ps1_receive(char* buf, size_t len) {
	int i = 0;

	//ps1_send("\003");
	while (i<len) {
		char c;
		while(read(ps1, &c, 1)<0);
		buf[i++] = c;
		if(c=='\n') {break;}
	}
	buf[i]='\0';
	plog("RECEIVED FROM PS1:\n---------------\n%s---------------\n",buf);
}

double ps1_getv(void) {
	usleep(200000);
	tcdrain(ps1);
	ps1_send("MEAS:VOLT? P6V\012");
	tcdrain(ps1);
	char buf[256];
	ps1_receive(buf, 256);
	return atof(buf);
}

double ps1_getc(void) {
	ps1_send("MEAS:CURR? P6V\n");
	char buf[256];
	ps1_receive(buf, 256);
	return atof(buf);
}

void ps1_setv(double volts) {
	ps1_send("DISP:TEXT 'now at %.5f'\n",volts);
	ps1_send("VOLT %.5f\n",volts);
}

void ps1_setc(double amps) {
	ps1_send("CURR %.5f\n",amps);
}

int main() {
	char buf[100];
	load_config();
	init_log();
	init_serial(&ps1, config_lookup(config, "ps1"));

	ps1_send("*RST;*CLS;\nSYST:REM\n");
	ps1_send("DISP:TEXT 'I am ps1.'\nINST:SEL P6V\nVOLT 0.0\n");
	usleep(800000);
	ps1_send("DISP:TEXT:CLE;:DISP ON\n");
	ps1_send("OUTPUT ON\n");

	int delay;
	config_lookup_int(config, "delay", &delay);
	float volts = 0.0;
	config_setting_t * profile = config_lookup(config, "profile");
	int iters = config_setting_length(profile);
	int i = 0;
	while(i<iters) {
		usleep(delay);
		volts = config_setting_get_float_elem(profile,i);
		ps1_setv(volts);
		double mv = ps1_getv(); double mc = ps1_getc();
		double nv = read_nidaq(&nidaq, config_lookup(config, "nidaq"));
		char buf[2048];
		sprintf(buf, "Measured voltage: %lf V\nMeasured current: %lf A\nMeasured power: %lf W\nNIDAQ averaged voltage: %lf\n", mv, mc, mv*mc, nv);
		plog("%s", buf);
		printf("%s", buf);
		pgraph(mv,mc,mv*mc,nv);
		i++;
	}
	ps1_send("VOLT 0.0\nOUTPUT OFF\nDISP:TEXT 'ps1 inactive'\n");
	return 0;
}
