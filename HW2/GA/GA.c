#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#define SWAP(x, y, t) ((t)=(x), (x)=(y), (y)= (t))
#define pi 3.14159
#define RULE 5
#define SET 10
#define crossover_rate 4 //1-10
#define generation 100
#define mutation_rate 10 // rand()%100
void ms_function_x(int s,float x);
void ms_function_fi(int s,float fi);
float calc_angle(int s);
void initialize(int s);
float docking_error(double fi,double x,double y);
void calc_weight_dock();
void select_better_set();
void copy_better_set();
void output_best_set();
void crossover();
void mutation();
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

//pool
float pma[SET][RULE];
float pta[SET][RULE];
float pmb[SET][RULE];
float ptb[SET][RULE];
float ptt[SET][RULE];
//dock
float dock[SET];
float weight_dock[SET];
int better_index[4];
int b[4];
//global min
float global_min = 100;
int main()
{
    srand((unsigned)time(NULL));
    float angle;
    float x,y,fi;
    int s = 0,temp = 0,step = 0,point = 0;
    double dock_temp;
    float x_temp,y_temp,fi_temp;

    //generate first parent

    for (s = 0; s < SET;s++) {
    initialize(s);
    dock_temp = 0;
    point = 0;
    for (x = 20; x <= 80; x = x + 10) {
        for (fi = -80; fi <= 260; fi = fi + 10) {
        y = 30;
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
   // printf("point = %d \n",point);
    dock[s] = dock_temp / point;
   // printf("dock[%d]=%f \n\n",s,dock[s]);
    }

    //calc weight dock
    calc_weight_dock();
    //search better set
    select_better_set();
    //printf("b1=%d b2=%d b3=%d b4=%d \n",b1,b2,b3,b4);
    copy_better_set();
    crossover();
	FILE *f_loop;
	f_loop = fopen("learning_curve.txt","w+t");    
    int g = 0; int z; float average_docking_error;
   while (g < generation) {
   for (s = 0; s < SET;s++) {
    dock_temp = 0;
    point = 0;
    for (x = 20; x <= 80; x = x + 10) {
        for (fi = -80; fi <= 260; fi = fi + 10) {
        y = 30;
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
   // printf("point = %d \n",point);
    dock[s] = dock_temp / point;

    }
    //calc weight dock
    calc_weight_dock();
    //search better set
    select_better_set();
    //printf("b1=%d b2=%d b3=%d b4=%d \n",b1,b2,b3,b4);
    copy_better_set();
    crossover();
    mutation();
    g++;
    printf("g= %d ",g);


    printf("minimum docking error = %f\n",dock[b[0]]);
    fprintf(f_loop,"minimum docking error = %f\n",dock[b[0]]);
    average_docking_error = 0;
    }
	fclose(f_loop);
    //for (s = 0; s < SET;s++) {
    //printf("dock[%d]=%f \n\n",s,dock[s]);
    //}

    select_better_set();
    output_best_set();

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
            ms_function_x(10,x);
            ms_function_fi(10,fi);
            angle = calc_angle(10);
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
    //printf("此次的docking_error = %f \n",trajectory_e);

     average_docking_and_trajectory_error();
    system("pause");
    return 0;
}
void initialize(int s)
{
    int i;
    for (i = 0; i < RULE; i++) {
        Ma[s][i] = rand()%99 + 1;
        do {Ta[s][i] =(float)(rand()%40)/10;}
        while (Ta[s][i] == 0);
        Mb[s][i] = rand()%359 - 89;
        do {Tb[s][i] =(float)(rand()%40)/10;}
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
void calc_weight_dock()
{
    int s;
    float denominator = 0;
    for ( s = 0 ; s< SET; s++) {
        denominator += 1 / dock[s];
    }
    //store weight

    for (s = 0; s< SET; s++) {
        weight_dock[s] = (1 / dock[s]) / denominator;
      //  printf("weight_dock[%d] =%f \n",s,weight_dock[s]);
    }
    float temp = 0;
    for ( s = 0 ; s< SET; s++) {
        temp += weight_dock[s];
    }
   // printf("temp =%f \n",temp);
}
void select_better_set()
{
    int i;
    float list[SET];
    for (i = 0; i < SET;i++) {
        list[i] = weight_dock[i];
    }

	int  j,min; float temp;
	for (i = 0; i < SET-1; i++) {
		min = i ; 						//index of min is i.
		for (j = i+1; j < SET; j++)
			if (list[j] <= list[min])
				min = j;

		SWAP(list[i], list[min], temp);
	}

	for (i = 0 ; i < SET; i++) {
        if (list[9] == weight_dock[i]) {
            b[0] = i;
            if (global_min > dock[i]) {
                    global_min = dock[i];
                for (j = 0; j < RULE; j++) {
                    Ma[10][j] = Ma[i][j];
                    Ta[10][j] = Ta[i][j];
                    Mb[10][j] = Mb[i][j];
                    Tb[10][j] = Tb[i][j];
                    Tt[10][j] = Tt[i][j];
                }
            }
        }
        if (list[8] == weight_dock[i]) {
            b[1] = i;
        }
        if (list[7] == weight_dock[i]) {
            b[2] = i;
        }
        if (list[6] == weight_dock[i]) {
            b[3] = i;
        }
	}
    printf("global_min = %f \n",global_min);
}
void copy_better_set()
{
    int i,j;
    for (j = 0; j < 4;j++) {
        for (i = 0; i < RULE;i++) {
            pma[j][i] = Ma[b[j]][i];
            pta[j][i] = Ta[b[j]][i];
            pmb[j][i] = Mb[b[j]][i];
            ptb[j][i] = Tb[b[j]][i];
            ptt[j][i] = Tt[b[j]][i];
            /*
            printf("pma[%d][%d]=%f ",j,i,pma[j][i]);
            printf("pta[%d][%d]=%f ",j,i,pta[j][i]);
            printf("pmb[%d][%d]=%f ",j,i,pmb[j][i]);
            printf("ptb[%d][%d]=%f ",j,i,ptb[j][i]);
            printf("ptt[%d][%d]=%f \n",j,i,ptt[j][i]);
            */
        }
    }

    for (j = 4; j < 6; j++) {
        for (i = 0; i < RULE;i++) {

            pma[j][i] = pma[0][i] + (rand()% 10-5);

            do {
            pta[j][i] = pta[0][i] +(float)(rand()%10)/10 - 0.5;
            } while(pta[j][i] == 0);

            pmb[j][i] = pmb[0][i] + (rand()%36 - 18);

            do {
            ptb[j][i] = ptb[1][i] + (float)(rand()%10)/10 - 0.5;
            } while (ptb[j][0] == 0);
            do {
            ptt[j][i] = ptt[0][i] + (rand()% 2 - 1);
            } while (ptt[j][i] == 0);

        }

    }
    for (j = 6; j < 8; j++) {
        for (i = 0; i < RULE;i++) {
            pma[j][i] = pma[1][i] + rand()% 20 - 10;
            do {
            pta[j][i] = pta[1][i] + (float)(rand()%10)/8 -0.625;
            } while(pta[j][i] == 0);
            pmb[j][i] = pmb[1][i] + rand()% 72 - 36;
            do {
            ptb[j][i] = ptb[1][i] + (float)(rand()%10)/8- 0.625;
            } while (ptb[j][i] == 0);
            do {
            ptt[j][i] = ptt[1][i] + rand()% 4 - 2;
            } while (ptt[j][i] == 0);
        }

    }

    for (i = 0; i < RULE;i++) {
            pma[8][i] = pma[2][i] + rand()% 30 - 15;
            do {
            pta[8][i] = pta[2][i] + (float)(rand()%10)/8 - 0.625;
            } while(pta[8][i] == 0);
            pmb[8][i] = pmb[2][i] + rand()% 108 - 54;
            do {
            ptb[8][i] = ptb[2][i] + (float)(rand()%10)/8- 0.625;
            } while (ptb[8][i] == 0);
            do {
            ptt[8][i] = ptt[2][i] + rand()% 8 - 4;
            } while (ptt[8][i] == 0);
    }
    for (i = 0; i < RULE;i++) {
            pma[9][i] = pma[3][i] + rand()% 40 - 20;
            do {
            pta[9][i] = pta[3][i] + (float)(rand()%10)/5 - 1;
            } while(pta[9][i] == 0);
            pmb[9][i] = pmb[3][i] + rand()% 144 - 72;
            do {
            ptb[9][i] = ptb[3][i] + (float)(rand()%10)/5- 1;
            } while (ptb[9][i] == 0);
            do {
            ptt[9][i] = ptt[3][i] + rand()% 16 - 8;
            } while (ptt[9][i] == 0);
    }
    /*
    for (j = 0; j < SET;j++) {
        for (i = 0; i < RULE;i++) {

            printf("pma[%d][%d]=%f ",j,i,pma[j][i]);
            printf("pta[%d][%d]=%f ",j,i,pta[j][i]);
            printf("pmb[%d][%d]=%f ",j,i,pmb[j][i]);
            printf("ptb[%d][%d]=%f ",j,i,ptb[j][i]);
            printf("ptt[%d][%d]=%f \n",j,i,ptt[j][i]);

        }
    }
    */
}
void crossover()
{
    int i,j,temp,s1 = 0; float s2;
    for (j = 0; j < SET;j++) {
        temp = rand()%10;
        while (s1 == j) {
            s1 = rand()%10;
        }

        if (temp < crossover_rate) {

            for (i = 0; i < RULE;i++) {
                s2 = (float)(rand()%6) / 10;
                Ma[j][i] =pma[j][i] + s2* (pma[j][i]-pma[s1][i]);
                s2 =(float)(rand()%20) / 10;
                Ta[j][i] =pta[j][i] + s2* (pta[j][i]-pta[s1][i]);
                s2 = (float)(rand()%6) / 10;
                Mb[j][i] =pmb[j][i] + s2* (pmb[j][i]-pmb[s1][i]);
                s2 = (float)(rand()%20) / 10;
                Tb[j][i] =ptb[j][i] + s2* (ptb[j][i]-ptb[s1][i]);
                s2 = (float)(rand()%6) / 10 ;
                Tt[j][i] =ptt[j][i] + s2* (ptt[j][i]-ptt[s1][i]);
            }

        }
        else {
            for (i = 0; i < RULE;i++) {
                Ma[j][i] =pma[j][i];
                Ta[j][i] =pta[j][i];
                Mb[j][i] =pmb[j][i];
                Tb[j][i] =ptb[j][i];
                Tt[j][i] =ptt[j][i];
            }
        }
    }
}

void mutation()
{
    int i,s,m;
    for (s = 0; s < SET; s++) {
            m = rand()%100;
        for (i = 0; i < RULE; i++) {
         if (m < mutation_rate) {
            Ma[s][i] =Ma[s][i] + rand()% 100 - 50;
            do {Ta[s][i] =Ta[s][i] + (float)(rand()%30)/10 - 1.5;}
            while (Ta[s][i] == 0);
            Mb[s][i] =Mb[s][i] +rand()%180 - 90;
            do {Tb[s][i] =Tb[s][i]+ (float)(rand()%30)/10 - 1.5;}
            while (Tb[s][i] == 0);

            do {Tt[s][i] =Tt[s][i]+ rand()%30 - 15;}
            while (Tt[s][i] == 0);
         }
        }
    }
}

void output_best_set()
{
	FILE *f_loop;
	f_loop = fopen("best_parameter.txt","w+t");
	int i;
	for (i = 0; i < 5;i++) {
	fprintf(f_loop,"X[%d] Mxi = %.3f Sxi = %.3f \n",i,Ma[b[0]][i],Ta[b[0]][i]);
	fprintf(f_loop,"p[%d] Mpi = %.3f Spi = %.3f \n",i,Mb[b[0]][i],Tb[b[0]][i]);
	fprintf(f_loop,"t[%d] theta = %.3f \n",i,Tt[b[0]][i]);
	}

	fclose(f_loop);
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
	double angle = 0,docking_e = 0,trajectory_e = 0,length_sum = 0 ,temp = 0;
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
					ms_function_x(10,x);
					ms_function_fi(10,fi);
					angle = calc_angle(10);
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
