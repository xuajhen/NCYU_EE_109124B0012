#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#define pi 3.14159265359

/*fuzzy variable*/
//輪胎轉向區間角度 
float NB,NM,NS,ZE,PS,PM,PB;
//車子角度分類 
float RB,RU,RV,VE,LV,LU,LB;
//車子x位置分類 
float LE,LC,CE,RC,RI; 
//FAM-bank matrix p.s row major
float fam[7][5];  
float theta[7][5];
int angle[7][5]={{5,6,6,7,7},  
			     {3,5,6,7,7},
			     {2,3,5,6,7},
			     {2,2,4,6,6},
			     {1,2,3,5,6},
			     {1,1,2,3,5},
			     {1,1,2,2,3}};
/*----------------*/
float x_rank_array[5];            //x相關位置陣列 
float fi_rank_array[7];           //fi相關位置陣列  
int x_rank_major,x_rank_second;   //兩個x相關位置陣列index 
int fi_rank_major,fi_rank_second; //兩個fi相關位置陣列index 
//計算x位置分類的關係程度
float region_x(float x);

//計算車子角度分類的關係程度
float region_fi(float fi);
void region_theta();

//計算x與車子角度的關係程度 (相乘)  
float region_fi_x();
//求出turn angle
float turn_angle();
//計算docking_error
float docking_error(double fi,double x,double y);
//計算trajectory_error
float trajectory_error(double length_sum,double initial_x,double initial_y); 

void average_docking_and_trajectory_error();

float min(float x,float y)
{
    if( x < y )
	return x ;
    return y ;
}
float max(float x,float y)
{
    if( x > y )
	return x ;
    return y ;
}
int main()
{
	double x,y,initial_x,initial_y,last_x,last_y;
	double fi;double angle;int step;
	double docking_e;
	double trajectory_e,length_sum;
	float x_rank_max,fi_rank_max;
	FILE *f_loop;
	f_loop = fopen("loop.txt","w+t"); 
	/*輸入*/
	do {
	printf("請輸入x座標:\n");
	scanf("%lf",&x);
	} while (x < 0 || x > 100);
	do {
	printf("請輸入y座標:\n");
	scanf("%lf",&y);
	} while (y < 0 || y > 100);
	do {
	printf("請輸入角度:\n");
	scanf("%lf",&fi);
	} while (fi < -90 || fi > 270); 
	fprintf(f_loop,"x = %.6f, y = %.6f, fi = %.6f (初始值)\n",x,y,fi);
	/*test x*/
	//printf("x_rank_max = %.3f\n",x_rank_max);
	//printf("LE=%.3f LC=%.3f CE=%.3f RC=%.3f RI=%.3f \n",LE,LC,CE,RC,RI);
	//
	/*test fi*/
	//printf("fi_rank_max = %.3f\n",fi_rank_max);
	//printf("RB=%.3f RU=%.3f RV=%.3f VE=%.3f LV=%.3f LU=%.3f LB=%.3f \n",RB,RU,RV,VE,LV,LU,LB);
	//		
	//
	//printf("%d\n",angle);
	
	//store initial x,y
	initial_x = x;
	initial_y = y;

	while (x > 0 && x < 100 && y > 0 && y < 100) {
		
		step++;
		printf("---------------step<%d>-----------------\n",step);
		last_x = x;
		last_y = y;
		
		x_rank_max = region_x(x);
		fi_rank_max = region_fi(fi);
		region_fi_x();
		region_theta();
		angle = turn_angle();
		fi += angle;
		x += cos(fi*pi/180);
		y += sin(fi*pi/180);
		
		length_sum += sqrt((last_x - x)*(last_x - x)+(last_y - y)*(last_y - y));
			/*print fam_bank*/
		printf("    LE    LC    CE    RC    RI\n");
		printf("RB  %.3f %.3f %.3f %.3f %.3f \n",fam[0][0],fam[0][1],fam[0][2],fam[0][3],fam[0][4]);
		printf("RU  %.3f %.3f %.3f %.3f %.3f \n",fam[1][0],fam[1][1],fam[1][2],fam[1][3],fam[1][4]);
		printf("RV  %.3f %.3f %.3f %.3f %.3f \n",fam[2][0],fam[2][1],fam[2][2],fam[2][3],fam[2][4]);
		printf("VE  %.3f %.3f %.3f %.3f %.3f \n",fam[3][0],fam[3][1],fam[3][2],fam[3][3],fam[3][4]);
		printf("LV  %.3f %.3f %.3f %.3f %.3f \n",fam[4][0],fam[4][1],fam[4][2],fam[4][3],fam[4][4]);
		printf("LU  %.3f %.3f %.3f %.3f %.3f \n",fam[5][0],fam[5][1],fam[5][2],fam[5][3],fam[5][4]);
		printf("LB  %.3f %.3f %.3f %.3f %.3f \n",fam[6][0],fam[6][1],fam[6][2],fam[6][3],fam[6][4]);
		printf("\n");
		printf("LE=%.3f LC=%.3f CE=%.3f RC=%.3f RI=%.3f \n",LE,LC,CE,RC,RI);
		printf("RB=%.3f RU=%.3f RV=%.3f VE=%.3f LV=%.3f LU=%.3f LB=%.3f \n\n",RB,RU,RV,VE,LV,LU,LB);
		printf("turn_angle = %f\n\n",angle);
		
		fprintf(f_loop,"x = %.6f, y = %.6f, fi = %.6f \n",x,y,fi);
		}
	printf("路徑檔案生成完成\n");
	fclose(f_loop);
	
	docking_e = docking_error(fi,x,y);
	trajectory_e = trajectory_error(length_sum,initial_x,initial_y);
	printf("此次的docking_error = %f \n",docking_e);
	printf("此次的trajectory_error = %f \n",trajectory_e);
	
	average_docking_and_trajectory_error();

	system("pause");
	return 0;
	
	
	
	
}
float region_x(float x)
{
	float rank_r,rank_l; // 方程式右邊 左邊 之相關程度 
	//Membership funtion of LE
	rank_r = (35-x)/(25); rank_l = 1;
	LE = max(min(rank_l,rank_r),0);
	x_rank_array[0] = LE;
	//Membership funtion of LC
	rank_r = (50-x)/(10); rank_l = (x-30)/(10);
	LC = max(min(rank_l,rank_r),0);
	x_rank_array[1] = LC;	
	//Membership funtion of CE
	rank_r = (55-x)/(5); rank_l = (x-45)/(5);
	CE = max(min(rank_l,rank_r),0);
	x_rank_array[2] = CE;
	//Membership funtion of RC
	rank_r = (70-x)/(10); rank_l = (x-50)/(10);
	RC = max(min(rank_l,rank_r),0);
	x_rank_array[3] = RC;
	//Membership funtion of RI
	rank_r = 1; rank_l = (x-65)/(25);
	RI = max(min(rank_l,rank_r),0);
	x_rank_array[4] = RI;
	
	/*return region*/
	int i;float rank = 0;
	for (i = 0; i < 5; i++) {
		if ( x_rank_array[i] > rank) {
			rank = x_rank_array[i];
			x_rank_major = i;
		}
		if (x_rank_array[i] > 0 && x_rank_array[i] > rank) {
			x_rank_second = i;
		}
	}
	//printf("x_rank_major = %d, x_rank_second =%d \n",x_rank_major,x_rank_second);
	//printf("rank = %.3f \n",rank);
	return rank;
} 

float region_fi(float fi)
{
	float rank_r,rank_l; // 方程式右邊 左邊 之相關程度 
	//Membership funtion of RB
	rank_r = (10-fi)/(55); rank_l = (fi+110)/(55);
	RB = max(min(rank_l,rank_r),0);
	fi_rank_array[0] = RB;
	//Membership funtion of RU
	rank_r = (60-fi)/(35); rank_l = (fi+10)/(35);
	RU = max(min(rank_l,rank_r),0);
	fi_rank_array[1] = RU;	
	//Membership funtion of RV
	rank_r = (90-fi)/(25); rank_l = (fi-40)/(25);
	RV = max(min(rank_l,rank_r),0);
	fi_rank_array[2] = RV;
	//Membership funtion of VE
	rank_r = (100-fi)/(10); rank_l = (fi-80)/(10);
	VE = max(min(rank_l,rank_r),0);
	fi_rank_array[3] = VE;
	//Membership funtion of LV
	rank_r = (140-fi)/(25); rank_l = (fi-90)/(25);
	LV = max(min(rank_l,rank_r),0);
	fi_rank_array[4] = LV;
	//Membership funtion of LU
	rank_r = (190-fi)/(30); rank_l = (fi-130)/(30);
	LU = max(min(rank_l,rank_r),0);
	fi_rank_array[5] = LU;
	//Membership funtion of LB
	rank_r = (280-fi)/(60); rank_l = (fi-165)/(60);
	LB = max(min(rank_l,rank_r),0);
	fi_rank_array[6] = LB;	
	
	/*return region*/
	int i;float rank = 0;
	for (i = 0; i < 7; i++) {
		if ( fi_rank_array[i] > rank) {
			rank = fi_rank_array[i];
			fi_rank_major = i;
		}
		if (fi_rank_array[i] > 0 && fi_rank_array[i] > rank) {
			fi_rank_second = i;
		}
	}
	//printf("fi_rank_major = %d, fi_rank_second =%d \n",fi_rank_major,fi_rank_second);
	//printf("rank = %.3f \n",rank);
	return rank;	
}

void region_theta()
{
	float theta_r,theta_l; // 方程式右邊 左邊 之相關程度 
	int code;
 	int row,col;
 	for (row = 0; row < 7 ; row++) {
 		for (col = 0; col < 5;col++) {
 			code = angle[row][col];
 			if (code == 1) {
 			//Membership funtion of NB
 			theta_r = -((fam[row][col])*15+15); theta_l = 0;
			NB = (theta_r + theta_l)/2;
			theta[row][col] = NB;
			}
			if (code == 2) {
			//Membership funtion of NM
			theta_r = -((fam[row][col])*10+5); theta_l = -((fam[row][col])*10+25);
			NM = (theta_r + theta_l)/2;
			theta[row][col] = NM;					
			}
			if (code == 3) {
			//Membership funtion of NS
			theta_r = -((fam[row][col])*6); theta_l = -((fam[row][col])*6+12);
			NS = (theta_r + theta_l)/2;
			theta[row][col] = NS;				
			}
			if (code == 4) {
			//Membership funtion of ZE
			theta_r= ((fam[row][col])*5); theta_l = -((fam[row][col])*5);
			ZE = (theta_r + theta_l)/2;
			theta[row][col] = ZE;				
			}
			if (code == 5) {
			//Membership funtion of PS
			theta_r = (-(fam[row][col])*6+12); theta_l = ((fam[row][col])*6);
			PS = (theta_r + theta_l)/2;
			theta[row][col] = PS;				
			}
			if (code == 6) {
			//Membership funtion of PM
			theta_r = (-(fam[row][col])*10+25); theta_l = ((fam[row][col])*10+6);
			PM = (theta_r + theta_l)/2;
			theta[row][col] = PM;				
			}
			if (code == 7) {
			//Membership funtion of PB
			theta_r = 0; theta_l = ((fam[row][col])*15+15);
			PB = (theta_r + theta_l)/2;
			theta[row][col] = PB;						
			}													
		 }
	}

}
float region_fi_x()
{
	int row,column;
	for (row = 0; row < 7; row++) {
		for (column =0; column < 5;column++) {
			fam[row][column] = x_rank_array[column]*fi_rank_array[row];
		}
	}

} 

float turn_angle()
{
	int row,column;
	
	//分子 w1z1+w2z2+...... w35z35
	float fraction = 0;
	for (row = 0; row < 7; row++) {
		for (column = 0; column < 5;column++) {
			fraction += fam[row][column]*theta[row][column];
		}
	}
	//分母 w1+w2+w3+.....+w35
	float denominator = 0; 
	for (row = 0; row < 7; row++) {
		for (column = 0; column < 5;column++) {
			denominator += fam[row][column];
		}
	}	 
	float angle;
	//printf("f =%.3f,d =%.3f ",fraction,denominator);
	angle = fraction / denominator;
	//printf("angle =%f\n",angle);
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
	double angle,docking_e,trajectory_e,length_sum;
	int step;
	step = 0;
	for(x_i = 20; x_i <= 80; x_i = x_i + 10) {
		for (y_i = 20; y_i <= 50; y_i = y_i + 10) {
			for (fi_i = -80; fi_i <= 260; fi_i = fi_i + 5) {
				x = x_i;
				y = y_i;
				fi= fi_i;	
				while (x > 0 && x < 100 && y > 0 && y < 100) {
					last_x = x;
					last_y = y;	
					region_x(x);
					region_fi(fi);
					region_fi_x();
					region_theta();
					angle = turn_angle();
					fi += angle;
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
