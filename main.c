#define F_CPU 8000000UL
#define MAX 100

#include <avr/io.h>
#include <avr/sfr_defs.h>
#include <util/delay.h>
#include <string.h>
#include <stdbool.h>
#include "USART_128.h"

#define Max	500   ////check Max and MAX
#define E_Max	1000

char a[MAX];
int sensor_weight[8] = {-6,-4,-2,0,0,2,4,6};
int i,m,j,n, mode=0, temp=0, s[8], h[8],k;
int error, des_dev, cur_dev;
float kp=25 , kd = 10 , ki = 0 , E_sum = 0 , e_old = 0, pid_out;
int dry_speed=600,test_speed=700; 


char path[100];//={'S','U','L','L','R','U','L','L','U','L','L','L'};
unsigned int pathLength = 0;
//int pathIndex;
//unsigned int status = 0;		//for differentiating

int readSensor();
void pwm_init();
float err_calc();
float pid(float);
void motorPIDcontrol(int);
void mazeSolve();
void runExtraInch();
void motor_stop();
void mazeEnd();
void goAndTurn(char);
void save_path(char);
void simplifyPath();
void got_itR();
void got_itL();
void got_itB();
void got_itl();


int main(void)
{
	//char q;
	//USART_Init(51,0);
	DDRF |= 0X00;
	DDRD |= 0X00;
	DDRB |= 0XFF;
	DDRA |= 0XFF;
	
	pwm_init();
	
    while(1)
    {
		mazeSolve();
    }
}
void pwm_init()
{
	TCCR1A |= (1<<COM1A1) | (1<<COM1B1) | (1<<WGM11);
	TCCR1B |= (1<<WGM12) | (1<<WGM13) | (1<<CS10);
	ICR1 = 4000;
}

int readSensor()
{
	s[8] = 0;
	k=0;
	for(i=0;i<8;i++)
	{
		if(bit_is_set(PIND,i))
		{
			s[i] = 1;
		}
		else
		{
			s[i] = 0;
		}
	}
	k = s[7] + s[6]*2 + s[5]*4 + s[4]*8 + s[3]*16 + s[2]*32 + s[1]*64 + s[0]*128; // left most ir s[0]
	return(k);
}

float err_calc()
{
	cur_dev=0;
	des_dev=0;
	h[8] = 0;
	m=0;
	
	for(i=1;i<7;i++)
	{
		if(bit_is_set(PIND,i))	
		{
			h[i] = 1;
		}			
		else
		{
			h[i] = 0;
			m++;
		}		
		cur_dev += h[i] * sensor_weight[i];
	}
	
	if(m==0)
	{m=1;}
	
	cur_dev = (cur_dev / m) * 4000;
	error = des_dev - cur_dev;
	return error;
}

float pid(float err)
{ 
	E_sum += err;
	if (E_sum > E_Max)
	{
		E_sum = E_Max;
	}
	else if (E_sum < -E_Max)
	{
		E_sum = -E_Max;
	}
	
	pid_out = (kp * err) + (ki * E_sum) + (kd * (err - e_old));
	e_old = err;
	
	if(pid_out > Max)
	{
		pid_out = Max; 
	}
	else if (pid_out < -Max)
	{
		pid_out = -Max;
	}
	return pid_out;
}

void motorPIDcontrol(int base)
{
	OCR1A = base + (pid(err_calc()));
	OCR1B = base - (pid(err_calc()));
	PORTA |= ((1<<PINA1) | (1<<PINA2));
}
/* middle 6 sensors for line following using pid concept generating error value  
and last 2 one will be for node recognition
*/


	//turn the bot while the deviation of the line sensor become approximately zero
	//motorPIDcontrol();

//first path function


void mazeSolve()
{
	int x=0;
	int c=0;
	
	if(bit_is_set(PINF,1))
	{
		x=1;
	}
	
	while(1)
	{
		mode = readSensor(); 
	    switch(mode)
		{
			case 00:
		//	case 195:
			//case 192:
			//case 3:
			//case 128:
			//case 1:
			motor_stop();
			runExtraInch();
			temp = 0;
			temp = readSensor();  

			if(temp == 0)
			{    
				
			    mazeEnd();
				//save_path('L');
			}
			else
			{
				OCR1A = 0;
				OCR1B = 700;
				_delay_ms(900);
				save_path('L');
				
			}
			break;
			
			case 255:				//no_line
				motor_stop();
				goAndTurn('B');	//turn,angle
			//_delay_ms(3000);
				//got_itU(readSensor());
				save_path('B');
			break;
			
			case 248:
			case 240:
			case 224:
			//case 192:
				motor_stop();
				runExtraInch();
				temp = 0;
				temp = readSensor();
				if(temp == 255) 
				{
					goAndTurn('R');
			//		_delay_ms(1300);
					//got_itR(readSensor());
					save_path('R');
				}     
				else 
				{
					motorPIDcontrol(dry_speed);
					save_path('S');
				}			
			break;
			   
			case 15:
			case 7:
			case 31:
			//case 1:
			case 3:
				//goAndTurn('l');
				//_delay_ms(1300);
				got_itl();
				save_path('L');
			break;
			
			case 231:
			case 207:
			case 243:
			case 199:
			case 227:
				motorPIDcontrol(dry_speed);			//calculatePID(); //motorPIDcontrol();
			break;
			
			default:
			motorPIDcontrol(dry_speed);
		}
		
	}
	/*while(x)
	{                                                           
		 mode = readSensor();
		 if(bit_is_clear(PIND,0) || bit_is_clear(PIND,7))
		 {
			char dir=path[c];
			switch(dir)
			{
				case 'L':
				goAndTurn('l');
				//_delay_ms(1300);
				//got_itL(readSensor());
				break;
				
				case 'R':
				goAndTurn('R');
				//_delay_ms(1300);
				//got_itR(readSensor());
				break;
				
				case 'S':
				motorPIDcontrol(test_speed);
				break;
				
				default:
				motorPIDcontrol(test_speed);
			}
			c++;		
		 }
		 else
		 {
		//	while((mode==231)||(mode==207)||(mode==243)||(mode==199)||(mode==227))
			//  {
			  motorPIDcontrol(dry_speed);
			  //}
		 }
	}*/
		//return z;
}

void runExtraInch()
{
	//motorPIDcontrol();
	OCR1A = 500;
	OCR1B = 500;
	_delay_ms(500);
	motor_stop();
}

void motor_stop()
{
	OCR1A = 0;
	OCR1B = 0;
	_delay_ms(500);
}

void mazeEnd()
{
	//while(!(bit_is_set(PINF,1)))    // close the switch after putting bot on start point......
	//{
		OCR1A = 0;
		OCR1B = 0;
		_delay_ms(10000);
	//}
	simplifyPath();
	_delay_ms(500);
}

void goAndTurn(char dir)
{
	switch(dir)
	{
		case 'R':
			//OCR1A=700;
			//OCR1B=0;
			got_itR();
			break;
		
		case 'L':
			//OCR1B=700;
			//OCR1A=0;
			got_itL();
			break;
			
		case 'B':
			PORTA &= ~(1<<PINA2);
			//OCR1A=400;
			//OCR1B=400;
			got_itB();
			break;
			
		case 'l':
			got_itl();
			break;	
			
		default:
		motorPIDcontrol(dry_speed);
	}				
}	


void save_path(char dir)
{
	path[pathLength] = dir;
	pathLength++;
	//return dir;
}



int arrShift(int i ,int len) 
{
	for(i=i ; i<len-2 ; i++)
	    {
		   a[i] = a[i+2];
    	}
	len = len-2;
	return len;
}

void simplifyPath()
{
	//scanf("%s",a);
	int len = strlen(path);
	int i;
	int check = 0;
	
	start :
	for(i=0 ; i<len-2 ; i++)
	 {
		if(path[i+1] == 'B')
		 {
			switch(path[i]) 
			{
				case 'L':
				switch(path[i+2])
				 {
					case 'L':
					path[i+2] = 'S';
					check = 1;
					break;
					
					case 'S':
					path[i+2] = 'R';
					check = 1;
					break;
					
					case 'R':
					path[i+2] = 'B';
					check = 1;
					break;
				}
				break;
				
				case 'S':
				switch(path[i+2]) 
				{
					case 'L':
					path[i+2] = 'R';
					check = 1;
					break;
					
					case 'S':
					path[i+2] = 'B';
					check = 1;
					break;
				}
				break;
				
				case 'R':
				switch(path[i+2]) 
				{
					case 'L':
					path[i+2] = 'B';
					check = 1;
					break;
				}
				break;
			}
				if(check == 1)
				 {
					len = arrShift(i,len);
					check = 0;
					//printf("\n");
					goto start;
				}
		}
	}
	
}

void got_itR()
{
	while(((n=readSensor() & (1<<3)) || (n=readSensor() & (1<<4))))
	{
			OCR1A=700;
			OCR1B=0;
			//n=readSensor();	
	}
}
void got_itB()
{
	while(((n=readSensor() & (1<<3)) || (n=readSensor() & (1<<4))))
	{
		OCR1A=700;
		OCR1B=700;
	}
}
void got_itl()
{
	while(!((n=readSensor() & (1<<3)) || (n=readSensor() & (1<<4))))
	{
		OCR1B=700;
		OCR1A=0;
	//	_delay_ms(1000);
	}
}
void got_itL()
{
	int q=0;
	if (!((n=readSensor() & (1<<3)) || (n=readSensor() & (1<<4))))
	{
		q=1;
	}
	while(q)
	{
		
		OCR1B=500;
		OCR1A=0;
		_delay_ms(100);
		if (!((n=readSensor() & (1<<3)) || (n=readSensor() & (1<<4))))
		{
			q=0;
		}
	}
}



// 
///if this fails then go for last ir and first ir