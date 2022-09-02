#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#define SWAP(x, y, t) ((t)=(x), (x)=(y), (y)= (t))
#define pi 3.14159
#define RULE 5
#define SET 10

#define generation 200 

void ms_function_x(int s,float x);
void ms_function_fi(int s,float fi);
float calc_angle(int s);
void initialize(int s);
float docking_error(double fi,double x,double y);
void calc_local_best();
void calc_global_best();
void crossover();

float trajectory_error(double length_sum,double initial_x,double initial_y);
void average_docking_and_trajectory_error();
//rule weight set
float Wa[SET][RULE];
float Wb[SET][RULE];
//rule parameter
float Ma[SET+1][RULE];
float Ta[SET+1][RULE];
float Mb[SET+1][RULE];
float Tb[SET+1][RULE];
float Tt[SET+1][RULE];

//past
float pma[SET][RULE];
float pta[SET][RULE];
float pmb[SET][RULE];
float ptb[SET][RULE];
float ptt[SET][RULE];

//min
float mma[RULE];
float mta[RULE];
float mmb[RULE];
float mtb[RULE];
float mtt[RULE];
//dock
float dock[SET];
double past_dock[SET];
double past_mindock = 100;
//global min
int main()
{
    srand((unsigned)time(NULL));
    float angle;
    float x,y,fi;
    int s = 0,temp,step,point;
    double dock_temp;
    float x_temp,y_temp,fi_temp;

    //generate first set

    for (s = 0; s < SET;s++) {
    initialize(s);
    past_dock[s] = 10;
    }

	FILE *f_loop;
	f_loop = fopen("learning_curve.txt","w+t");
    int g = 0; int z;
    while (g < generation) {
        for (s = 0; s < SET;s++) {
            dock_temp = 0;
            point = 0;
            for (x = 20; x <= 80; x = x + 10) {
                for (y = 20; y <= 50; y = y + 10) {
                    for (fi = -80; fi <= 260; fi = fi + 10) {
                        x_temp = x;
                        y_temp = y;
                        fi_temp = fi;
                        step = 0;
                        while (step < 200 && x_temp > 0 && x_temp < 100 && y_temp > 0 && y_temp < 100) {
                            ms_function_x(s,x_temp);
                            ms_function_fi(s,fi_temp);
                            angle = calc_angle(s);
                            fi_temp += angle;
                            if (fi_temp >= 360 || fi_temp <= -360) {
                                temp = fi_temp / 360;
                                fi_temp = fi_temp - 360*temp;
                            }

                            x_temp += cos(fi_temp*pi/180);
                            y_temp += sin(fi_temp*pi/180);
                            //printf("x = %f,y = %f,fi = %f \n",x_temp,y_temp,fi_temp);
                            step++;
                        }
                    point++;
                    dock_temp += docking_error(fi_temp,x_temp,y_temp);
                    //printf("x = %f,y = %f,fi = %f \n",x_temp,y_temp,fi_temp);
                    }
                }

            }
            // printf("point = %d \n",point);
            dock[s] = dock_temp / point;
            //printf("dock %f\n",dock[s]);
        }

        calc_local_best();
        calc_global_best();
        crossover();

        g++;
        printf("g= %d ",g);


        printf("minimum docking error = %f\n",past_mindock);
        fprintf(f_loop,"minimum docking error = %f\n",past_mindock);

    }

	fclose(f_loop);
    //for (s = 0; s < SET;s++) {
    //printf("dock[%d]=%f \n\n",s,dock[s]);
    //}
    int i;
	for (i = 0 ; i < RULE;i++) {
	Ma[SET+1][i] = mma[i]; 
	Ta[SET+1][i] = mta[i];
	Mb[SET+1][i] = mmb[i];
	Tb[SET+1][i] = mtb[i];
	Tt[SET+1][i] = mtt[i];		
	}


    do {
	printf("請輸入x座標:\n");
	scanf("%f",&x);
	} while (x < 0 || x > 100);
	do {
	printf("請輸入y座標:\n");
	scanf("%f",&y);
	} while (y < 0 || y > 100);
	do {
	printf("請輸入角度:\n");
	scanf("%f",&fi);
	} while (fi < -90 || fi > 270);

    float docking_e,trajectory_e,last_x,last_y,length_sum,initial_x,initial_y;
    initial_x = x;
    initial_y = y;
    printf("x = %f,y = %f,fi = %f \n",initial_x,initial_y,fi);
    step = 0;
        while (step < 200 && x > 0 && x< 100 && y > 0 && y < 100) {
            last_x = x;
            last_y = y;
            ms_function_x(SET+1,x);
            ms_function_fi(SET+1,fi);
            angle = calc_angle(SET+1);
            fi+= angle;
            if (fi>= 360 || fi <= -360) {
                temp = fi / 360;
                fi = fi- 360*temp;
            }
            x+= cos(fi*pi/180);
            y += sin(fi*pi/180);
            length_sum += sqrt(pow(last_x - x,2)+pow(last_y - y,2));
            printf("x = %f,y = %f,fi = %f \n",x,y,fi);
            step++;
            }
    docking_e = docking_error(fi,x,y);
    //trajectory_e = trajectory_error(length_sum,initial_x,initial_y);
    printf("此次的docking_error = %f \n",docking_e);
    //printf("此次的trajectory_error = %f \n",trajectory_e);

    average_docking_and_trajectory_error();
    system("pause");
    return 0;
}
void initialize(int s)
{
    int i;
    for (i = 0; i < RULE; i++) {
        Ma[s][i] = rand()%99 + 1;
        do {Ta[s][i] =(float)(rand()%30)/10;}
        while (Ta[s][i] == 0);
        Mb[s][i] = rand()%359 - 89;
        do {Tb[s][i] =(float)(rand()%30)/10;}
        while (Tb[s][i] == 0);

        do {Tt[s][i] = rand()%60 - 30;}
        while (Tt[s][i] == 0);
        /*
        printf("ma[%d]=%f ",i,Ma[s][i]);
        printf("ta[%d]=%f ",i,Ta[s][i]);
        printf("mb[%d]=%f ",i,Mb[s][i]);
        printf("tb[%d]=%f ",i,Tb[s][i]);
        printf("tt[%d]=%f \n",i,Tt[s][i]);
        */
    }
}

void ms_function_x(int s,float x)
{
    int i;
    for (i = 0; i < RULE ; i++) {
    Wa[s][i] = exp(-(pow(x-Ma[s][i],2)) / (pow(Ta[s][i],2)));
    //printf("Wa[%d]=%f ",i,Wa[0][i]);
    }
    //printf("\n");
}
void ms_function_fi(int s,float fi)
{
    int i;
    for (i = 0; i < RULE ; i++) {
    Wb[s][i]= exp(-(pow(fi-Mb[s][i],2)) / (pow(Tb[s][i],2)));
    }
}

float calc_angle(int s)
{
    int i;
	float fraction = 0,denominator = 0;
	float angle;
    for (i = 0; i < RULE; i++) {
		 	fraction += Wa[s][i]*Wb[s][i]*Tt[s][i];
		 	denominator += Wa[s][i]*Wb[s][i];
		}
	//	if (fraction <= 0.001) {
    //        fraction = 0.001;
	//	}
		if (denominator <= 0.001) {
            denominator = 0.001;
		}
	angle = fraction / denominator;
	return angle;
}
float docking_error(double fi,double x,double y)
{
	float d_error,fi_error,x_error,y_error;
	fi_error = ((90-fi)/180)*((90-fi)/180);

	x_error = ((50-x)/50)*((50-x)/50);
	y_error = ((100-y)/100)*((100-y)/100);
	d_error =sqrt((fi_error + x_error + y_error));
	return d_error;
}
void calc_local_best()
{
    int s,i;
    for (s = 0; s < SET;s++) {
            //printf("dock %f\n",dock[s]);
        if (dock[s] < past_dock[s]) {
            past_dock[s] = dock[s];
            for (i = 0; i < RULE; i++) {
                pma[s][i] = Ma[s][i];
                pta[s][i] = Ta[s][i];
                pmb[s][i] = Mb[s][i];
                ptb[s][i] = Tb[s][i];
                ptt[s][i] = Tt[s][i];
            }
        }
    }

}
void calc_global_best()
{
    int s,i;
    for (s = 0; s < SET;s++) {
        if (past_dock[s] < past_mindock) {
            past_mindock = past_dock[s];
            for (i = 0; i < RULE; i++) {
                mma[i] = pma[s][i];
                mta[i] = pta[s][i];
                mmb[i] = pmb[s][i];
                mtb[i] = ptb[s][i];
                mtt[i] = ptt[s][i];
            }
        }
    }
    //printf("past_mindock = %f \n",past_mindock);
}
void crossover()
{
    int s,i;
    float vector = 0;
    float w = 0.8,w1 = 1,w2 = 3,p1,p2;
    for (s = 0; s < SET;s++) {
        for (i = 0; i < RULE;i++) {
            p1 = (float)(rand()%1000) / 1000;
            p2 = (float)(rand()%1000) / 1000;
            //printf("p1 =%f,p2 =%f \n",p1,p2);
            //ma
            vector = w*vector + w1*p1*(pma[s][i]-Ma[s][i]) + w2*p2*(mma[i]-Ma[s][i]);
            if (vector > 5) {vector = 5;}
            if (vector < -5) {vector = -5;}
            Ma[s][i] = vector + Ma[s][i];

            vector = 0;
            p1 = (float)(rand()%1000) / 1000;
            p2 = (float)(rand()%1000) / 1000;    
            //ta30 
            vector = w*vector + w1*p1*(pta[s][i]-Ta[s][i]) + w2*p2*(mta[i]-Ta[s][i]);
            if (vector > 2) {vector = 2;}
            if (vector < -2) {vector = -2;}
            Ta[s][i] = vector + Ta[s][i];

            vector = 0;
            p1 = (float)(rand()%1000) / 1000;
            p2 = (float)(rand()%1000) / 1000;         
            //mb
            vector = w*vector + w1*p1*(pmb[s][i]-Mb[s][i]) + w2*p2*(mmb[i]-Mb[s][i]);
            if (vector > 10) {vector = 10;}
            if (vector < -10) {vector = -10;}
            Mb[s][i] = vector + Mb[s][i];

            vector = 0;
            p1 = (float)(rand()%1000) / 1000;
            p2 = (float)(rand()%1000) / 1000;           
            //tb
            vector = w*vector + w1*p1*(ptb[s][i]-Tb[s][i]) + w2*p2*(mtb[i]-Tb[s][i]);
            if (vector > 2) {vector = 2;}
            if (vector < -2) {vector = -2;}
            Tb[s][i] = vector + Tb[s][i];
           
            vector = 0;
            p1 = (float)(rand()%1000) / 1000;
            p2 = (float)(rand()%1000) / 1000;           
            //tt
            vector = w*vector + w1*p1*(ptt[s][i]-Tt[s][i]) + w2*p2*(mtt[i]-Tt[s][i]);
            if (vector > 5) {vector = 5;}
            if (vector < -5) {vector = -5;}
            Tt[s][i] = vector + Tt[s][i];
        
            vector = 0;
        }
    }
}



float trajectory_error(double length_sum,double initial_x,double initial_y)
{
	float t_error,length_short;

	length_short = sqrt((initial_x - 50)*(initial_x - 50)+(initial_y - 100)*(initial_y - 100));
	//printf("length_sum = %f \n",length_sum);
	//printf("length_short = %f \n",length_short);
	t_error = length_sum / length_short;

	return t_error;
}
void average_docking_and_trajectory_error()
{
	double x,y,x_i,y_i,last_x,last_y;
	double fi,fi_i;
	double angle,docking_e,trajectory_e,length_sum,temp;
	int step;
	step = 0;
	for(x_i = 25; x_i <= 75; x_i = x_i + 10) {
		for (y_i = 25; y_i <= 45; y_i = y_i + 10) {
			for (fi_i = -75; fi_i <= 255; fi_i = fi_i + 10) {
				x = x_i;
				y = y_i;
				fi= fi_i;
				while (x > 0 && x < 100 && y > 0 && y < 100) {
					last_x = x;
					last_y = y;
					ms_function_x(SET+1,x);
					ms_function_fi(SET+1,fi);
					angle = calc_angle(SET+1);
                    fi+= angle;
                    if (fi>= 360 || fi <= -360) {
                        temp = fi / 360;
                        fi = fi- 360*temp;
                    }
					x += cos(fi*pi/180);
					y += sin(fi*pi/180);
					length_sum += sqrt((last_x - x)*(last_x - x)+(last_y - y)*(last_y - y));
				}
				trajectory_e += trajectory_error(length_sum,x_i,y_i);
				length_sum = 0;
			  	docking_e += docking_error(fi,x,y);
			  	step++;
			}
		}
	}

	printf("資料筆數=%d\n",step);
	printf("averge docking error = %f\n",docking_e/step);
	printf("averge trajectory error = %f\n",trajectory_e/step);


}
