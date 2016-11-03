#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define DEBUG_SESSION 1
#define DEBUG_PRINT(...) fprintf(stderr, __VA_ARGS__);
#define DEBUG(...) if(DEBUG_SESSION) { DEBUG_PRINT(__VA_ARGS__) }

struct PID_data
{
    double KP;
    double KI;
    double KD;
    double min;
    double max;
    double target;
    double sum_error;
    double last_error;
    int debug;
    char * id;
};

struct PID_data * new_PID_data(char * label)
{
    struct PID_data * new = (struct PID_data *) malloc(sizeof(struct PID_data));
    new->sum_error = 0;
    new->last_error = 0;
    new->id = label;
    new->debug = 0;
    return new;
}

double PID_normalize_output(double output, double min, double max, char * id, int debug)
{
    double normalized = output < min ? min : (output > max ? max : output);    
    if(debug) 
    {
        DEBUG("%s : normalized %f\n", id, normalized);
    }
    return normalized;
}

double PID_KP_round(double KP, double error, char * id, int debug)
{
    double output = KP * error;   
    if(debug) 
    {
        DEBUG("%s : KP_round   %f\n", id, output);
    }
    return output;
}

double PID_KD_round(double KD, double error, double * last_error, char * id, int debug)
{
    double output = KD * (error - *last_error);
    *last_error = error;    
    if(debug) 
    {
        DEBUG("%s : KD_round   %f\n", id, output);
    }
    return output;
}

double PID_KI_round(double KI, double error, double * sum_error, char * id, int debug)
{
    *sum_error = error + (*sum_error);
    double output = KI * (*sum_error);   
    if(debug) 
    {
        DEBUG("%s : KI_round   %f\n", id, output);
    }
    return output;
}

double PID_round(struct PID_data * data, double current)
{
    double error = data->target - current;
    
    if(data->debug) 
    {       
        DEBUG("%s : target     %f\n", data->id, data->target);
        DEBUG("%s : current    %f\n", data->id, current);
        DEBUG("%s : error      %f\n", data->id, error);
        DEBUG("%s : last_error %f\n", data->id, data->last_error);
    }
    
    double output = PID_KP_round(data->KP, error, data->id, data->debug)
                  + PID_KD_round(data->KD, error, &data->last_error, data->id, data->debug)
                  + PID_KI_round(data->KI, error, &data->sum_error, data->id, data->debug);
    
    if(data->debug) 
    {
        DEBUG("%s : output     %f\n", data->id, output);
    }
    return PID_normalize_output(output, data->min, data->max, data->id, data->debug);
}

int main()
{
	exit(0);
}
