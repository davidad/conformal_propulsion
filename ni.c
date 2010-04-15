#include "NIDAQmxBase.h"
#include <stdio.h>

#define DAQmxErrChk(functionCall) { if( DAQmxFailed(error=(functionCall)) ) { goto Error; } }

int main (int argc, char *argv[])
{
   // Task parameters
    int32       error = 0;
    TaskHandle  taskHandle = 0;
    char        errBuff[2048]={'\0'};
    int32       i;

    // Channel parameters
    char        chan[] = "Dev1/ai0";
    float64     min = -10.0;
    float64     max = 10.0;

    // Timing parameters
    char        source[] = "OnboardClock";
    uInt64      samplesPerChan = 100;
    float64     sampleRate = 1200.0;

    // Data read parameters
    #define     bufferSize (uInt32)1000
    float64     data[bufferSize];
    int32       pointsToRead = -1;
    int32       pointsRead;
    float64     timeout = 10.0;

    while(1) {
	printf("new task\n\n");
    DAQmxErrChk (DAQmxBaseCreateTask ("", &taskHandle));
    DAQmxErrChk (DAQmxBaseCreateAIVoltageChan (taskHandle, chan, "", DAQmx_Val_NRSE, min, max, DAQmx_Val_Volts, NULL));
    DAQmxErrChk (DAQmxBaseCfgSampClkTiming (taskHandle, source, sampleRate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps, samplesPerChan));

    DAQmxErrChk (DAQmxBaseStartTask (taskHandle));

    DAQmxErrChk (DAQmxBaseReadAnalogF64 (taskHandle, pointsToRead, timeout, 0, data, bufferSize, &pointsRead, NULL));

    printf("pointsRead: %d\n",pointsRead);
    // Just print out the first 10 points
    for (i = 0; i < 8; ++i) {
       printf ("%f\n", data[i]);
    }
       DAQmxBaseStopTask (taskHandle);
       DAQmxBaseClearTask (taskHandle);
    }

Error:
    if (DAQmxFailed (error))
       DAQmxBaseGetExtendedErrorInfo (errBuff, 2048);

    if (taskHandle != 0)
    {
       DAQmxBaseStopTask (taskHandle);
       DAQmxBaseClearTask (taskHandle);
    }

    if (error)
	   printf ("DAQmxBase Error %d: %s\n", error, errBuff);

    return 0;
}
