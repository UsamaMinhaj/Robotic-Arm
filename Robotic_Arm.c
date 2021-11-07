#include<stddef.h>
#include"stm32f10x.h"
#define SaveLimit 12
#define ArrValue 22499
#define Prescalar 63

char Received;
char Input[4];
unsigned char counter=0;
unsigned char  Save=0;
unsigned char  Save_count=0;
unsigned char no_saved=0;
unsigned char Exception=0;
unsigned char CurrentPosA;
unsigned char CurrentPosB;
unsigned char CurrentPosC;
unsigned char CurrentPosD;
//Double array for storing input
char SavedInput[12][4];

//Initially status is 3 to indicate that it is not initialized
unsigned char Status=3;
unsigned char ProcessInputCoded();
void ProcessInputKeys();
void TransmitData();
void TransmitInteger(unsigned char);
void SavedData();
void Menu();
void Process_Saved();


void delay(unsigned int nCount)
{
	unsigned int i, j;

	for (i = 0; i < nCount; i++)
		for (j = 0; j < 0x2AFF; j++);
}


void PWM2_set( int rotation)
{
	TIM3->PSC=Prescalar;
		TIM3->ARR=ArrValue;
		TIM3->CCR1=(1055*rotation)/100 + 900;
		TIM3->CCR2=(1055*rotation)/100 + 900;
}
void PWM2_initialise(int prescaler, int period, int rotation)
{
	RCC->APB2ENR|=RCC_APB2ENR_IOPAEN;
	//RCC->AHBENR |= RCC_AHBENR_SDIOAEN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	GPIOA->CRL |= GPIO_CRL_MODE6_1;
	GPIOA->CRL |= GPIO_CRL_MODE6_0;
	GPIOA->CRL &= ~(GPIO_CRL_CNF6_0);
	GPIOA->CRL |= GPIO_CRL_CNF6_1;

	GPIOA->CRL |= GPIO_CRL_MODE7_1;
		GPIOA->CRL |= GPIO_CRL_MODE7_0;
		GPIOA->CRL &= ~(GPIO_CRL_CNF7_0);
		GPIOA->CRL |= GPIO_CRL_CNF7_1;

	//AFIO->EVCR=0b100001011;
	PWM2_set(rotation);
	TIM3->CCMR1 |=TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE;
	TIM3->CCMR1 |=TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2PE;
	TIM3->CCER |= TIM_CCER_CC1E;
	TIM3->CCER |= TIM_CCER_CC2E;
	TIM3->BDTR |= TIM_BDTR_MOE;
	TIM3->CR1 |= TIM_CR1_CEN;
	TIM3->EGR |= TIM_EGR_UG;


}

void PWM_set(int rotation)
{
	TIM1->ARR = ArrValue;
	TIM1->PSC = 63;
		delay(100);
	//if(rotation>=0 && rotation <=180)
			{
				//(5 + rotation/180*5) is the duty cycle required
			//For big servo
			TIM1->CCR1  = (25*rotation)/2 + 1000; //.5 is added for the sake of approximation

			TIM1->CCR4  =  (25*rotation)/2 + 1000;

			}
			delay(100);

}

void PWM_initialise(int prescaler, int period, int rotation)
{
	RCC->APB2ENR|=RCC_APB2ENR_IOPAEN;
	//RCC->AHBENR |= RCC_AHBENR_SDIOAEN;
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

			GPIOA->CRH |= GPIO_CRH_MODE8_1;
					GPIOA->CRH |= GPIO_CRH_MODE8_0;
					GPIOA->CRH &= ~(GPIO_CRH_CNF8_0);
					GPIOA->CRH |= GPIO_CRH_CNF8_1;



	GPIOA->CRH |= GPIO_CRH_MODE11_1;
		GPIOA->CRH |= GPIO_CRH_MODE11_0;
		GPIOA->CRH &= ~(GPIO_CRH_CNF11_0);
		GPIOA->CRH |= GPIO_CRH_CNF11_1;

	//AFIO->EVCR=0b100001011;
	PWM_set(rotation);

	TIM1->CCMR2 |=TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4PE;
	TIM1->CCMR1 |=TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE;

	TIM1->CCER |=  TIM_CCER_CC4E | TIM_CCER_CC1E;
	TIM1->BDTR |= TIM_BDTR_MOE;
	TIM1->CR1 |= TIM_CR1_CEN;
	TIM1->EGR |= TIM_EGR_UG;
}


//Delay function
void Delay_ms(uint16_t ms)
{
	for(int i=0;i<ms;i++)
	for(int j=0;j<36;j++)
	for(int k=0;k<1000;k++);
}


void UART1_interrupt_init()
{

	//Interrupt bit enable
	USART1->CR1 |= USART_CR1_RXNEIE;



	//ENABLE GLOBAL INTERRUPT FOR USART1
		NVIC_EnableIRQ(USART1_IRQn);
}

void UART1_init()
{
	//Initialize USART
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPAEN;
	//Using Alternate function push and pull
	GPIOA->CRH |= GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9_1 | GPIO_CRH_MODE9_0;
	GPIOA->CRH |= GPIO_CRH_CNF10_0;
	GPIOA->CRH &= ~(GPIO_CRH_CNF10_1);
	//Now enable bits for UART
	USART1->CR1 |= USART_CR1_UE  | USART_CR1_RE ;

	//BAUD RATE OF 9600
	USART1->BRR = (0x1D4C);
}

//Connected to Channel 1 of Timer 1 (Gripper Motor)
void Servo_RotateA(unsigned char rotation)
{
	TIM3->PSC=Prescalar;
			TIM3->ARR=ArrValue;
	if(rotation<0)
		rotation=0;
	if(rotation > 135)
		rotation = 135;
	//TIM1->CCR1  =  (5 + rotation/180*5)*1000/100 - 1 + .5; //.5 is added for the sake of approximation
	TransmitData("\n A Servo rotation ");
	TIM3->CCR1=(1055*rotation)/100 + 900;
	CurrentPosA = rotation;
}


//Connected to Channel 2 of Timer 1 (Elbow Motor)
void Servo_RotateB(unsigned char rotation)
{
	if(rotation<0)
		rotation=0;
	if(rotation > 135)
		rotation = 135;
	//TIM1->CCR2  =  (5 + rotation/180*5)*1000/100 - 1 + .5; //.5 is added for the sake of approximation
	TransmitData(" B Servo rotation ");
	TIM3->PSC=Prescalar;
	TIM3->ARR=ArrValue;
	TIM3->CCR2=(1055*rotation)/100 + 900;
	CurrentPosB = rotation;
}


//Connected to Channel 3 of Timer 1 (Shoulder Motor)
void Servo_RotateC(unsigned char rotation)
{
	if(rotation<0)
		rotation=0;
	if(rotation > 135)
		rotation = 135;
	//TIM1->CCR3  =  (5 + rotation/180*5)*1000/100 - 1 + .5; //.5 is added for the sake of approximation
	TransmitData(" C Servo rotation ");
	//(Input[1]-'0')*100 + (Input[2]-'0')*10 + (Input[3] - '0');
	if(rotation==120)
	{
		TransmitData("Rotation 120 ");
	}
	TIM3->PSC=Prescalar;
			TIM3->ARR=ArrValue;
			TIM1->CCR4=(25*rotation)/2 + 1000;
	CurrentPosC = rotation;

}


//Connected to Channel 4 of Timer 1 (Base Motor)
void Servo_RotateD(unsigned char rotation)
{
	if(rotation<0)
		rotation=0;
	if(rotation > 135)
		rotation = 135;
	TIM3->PSC=Prescalar;
	TIM3->ARR=ArrValue;
	TIM1->CCR1=(25*rotation)/2 + 1000;
	//TIM1->CCR4  =  (5 + rotation/180*5)*1000/100 - 1 + .5; //.5 is added for the sake of approximation
	TransmitData(" D Servo rotation");
	CurrentPosD = rotation;
}


int main()
{

	//Initialize USART1
	PWM_initialise(63,22499,0);
	PWM2_initialise(63,22499,0);
	UART1_init();
	Menu();
	Delay_ms(100);
	UART1_interrupt_init();
	while(1)
	{
		//Add some delay before

		//TransmitData("\nOsama\n");
	}
}


void TransmitData(char *Data  )
{

	int size = 0;
	USART1->CR1 |= USART_CR1_TE;
	Delay_ms(5);
	while(*(Data+size))
		size = size + 1;
	for(int i=0;i<size+1;i++)//Do not want to transmit NULL character
	{
	//Wait until Transmit register is empty
	while(!(USART1->SR & USART_SR_TXE));
			USART1->DR = Data[i];

	}
	USART1->CR1 &= ~(USART_CR1_TE);
}

void TransmitInteger(unsigned char Data)
{
	//Convert Integer into character
	char temp = Data + '0';
	USART1->CR1 |= USART_CR1_TE;
	Delay_ms(25);
	while(!(USART1->SR & USART_SR_TXE));
	USART1->DR = temp;
	USART1->CR1 &= ~(USART_CR1_TE);
}

void Key()
{
	if(Exception == 0)
		{
			Exception++;
			TransmitData("\nPlease input required key: ");
			USART1->SR &= ~(USART_SR_RXNE) ;
			return;
		}
	Received = USART1->DR;

	//Now look for cases
	switch(Received)
	{
	case 'W':
		Servo_RotateB(CurrentPosB+2);
		break;
	case 'S':
		Servo_RotateB(CurrentPosB-2);
			break;
	case 'A':
		Servo_RotateD(CurrentPosD+2);
			break;
	case 'D':
		Servo_RotateD(CurrentPosD-2);
			break;
	case 'I':
		Servo_RotateC(CurrentPosC+2);
			break;
	case 'K':
		Servo_RotateC(CurrentPosC-2);
			break;
	case 'J':
		Servo_RotateA(CurrentPosA+2);
			break;
	case 'L':
		Servo_RotateA(CurrentPosA-2);
			break;
	case 'X':
		Menu();
			break;
	}
}

void Coded()
{

	//NOt anything logical just to cater an internal error of USART1
	if(Exception == 0)
		{
			Exception++;
			TransmitData("\nNow give the code: ");
			USART1->SR &= ~(USART_SR_RXNE) ;
			return;
		}

				TransmitData("  ");
				TransmitInteger(counter + 1);

				Received = USART1->DR ;

				if(Received == 'X')
					{
						counter=0;
						Menu();
					}
				else if(Received == 'N' && counter == 4)
				{
					counter = 0;
					//Now Process the input

					//To check for error in rotation
				 ProcessInputCoded();

				}
				else
				{
					if(counter<4)
					{
						//Add to data buffer
						Input[counter] = Received;
						counter++;
					}
					else
					{
						//Transmit error because incorrect code has been sent
						TransmitData("Last Error! ");
						counter = 0;
						//USART1->CR1 &= ~(USART_CR1_RXNEIE);

						//USART1->SR &= 0XFFFFFFD7 ;
					}
				}



}

void USART1_IRQHandler()
{
	//if(  == 1)
	{
		if(Status == 1)
		{
			Key();
		}
		else if(Status == 2)
		{

			if(Save==2)
			SavedData();
			else if(Save==1)
			Coded();
		}

	}

}

void SavedData()
{
	//NOt anything logical just to cater an internal error of USART1
	if(Exception == 0)
	{
		Exception++;
		TransmitData("\nNow give the code: ");
		USART1->SR &= ~(USART_SR_RXNE) ;
		return;
	}

	//To check whether desired number of inputs have been saved
	if(Save_count<=no_saved )
	{


			TransmitData("  ");
			TransmitInteger(counter + 1);

			Received = USART1->DR ;
			if(Received == 'X')
				{
					counter=0;
					Menu();
				}
			else if(Received == 'N' && counter == 4)
			{
				counter = 0;
				//Now Process the input

				//To check for error in rotation
				unsigned char temp = ProcessInputCoded();
				if(temp == 1)
				{
					for(int i=0;i<4;i++)

				{
					SavedInput[Save_count][i] = Input[i];
				}
				Save_count++;
				if(Save_count==no_saved)
				{
					USART1->CR1 &= ~(USART_CR1_RXNEIE);
					TransmitData("\n Input Disabled. ");
					TransmitData("\n Now in repetition mode. ");
					Process_Saved();
					Save_count=0;
				}
				}

			}
			else
			{
				if(counter<4)
				{
					//Add to data buffer
					Input[counter] = Received;
					counter++;
				}
				else
				{
					//Transmit error because incorrect code has been sent
					TransmitData("Last Error! ");
					counter = 0;
					//USART1->CR1 &= ~(USART_CR1_RXNEIE);

					//USART1->SR &= 0XFFFFFFD7 ;
				}
			}
	}
	else
	{
		//Disable interrupt and stop taking input
		USART1->CR1 &= ~(USART_CR1_RXNEIE);
		TransmitData("\n Input Disabled. ");
		TransmitData("\n Now in repetition mode. ");
		Process_Saved();
	}
}


void Menu()
{

	Exception = 0;

	TransmitData("\nPlease choose one of the options for movement of Robotic Arm:  ");
	TransmitData("\n1- Arrow Keys  ");
	TransmitData("\n2- Code ");
	//Disable interrupt if one is there
	USART1->CR1 &= ~(USART_CR1_RXNEIE);
	//Keep receiving until correct input
	do
	{
	//Wait until data is received by polling
	while(!(USART1->SR & USART_SR_RXNE) );
	Status = USART1->DR;
	Status -= '0';
	//TransmitData("\nHello ");
	}while(!(Status  == 1 || Status  == 2));

	if(Status == 2)
	{
		TransmitData("\nPlease choose one of the options:  ");
			TransmitData("\n1- Runtime ");
			TransmitData("\n2- Save ");
			do
				{

				//Wait until data is received by polling
				while(!(USART1->SR & USART_SR_RXNE) );
				Save = USART1->DR;
				Save -= '0';
				//TransmitData("\nHello ");
				}while(!(Save  == 1 || Save  == 2));
		if(Save==2)
		{
			TransmitData("\nPlease input number of repetitions: ");

							do
							{
							//Wait until data is received by polling
							while(!(USART1->SR & USART_SR_RXNE) );
							no_saved = USART1->DR;
							no_saved -= '0';
							//TransmitData("\nHello ");
							}while((no_saved <= 0 || (no_saved  > (SaveLimit+1))));
		}

	}
	Delay_ms(20);
	USART1->CR1 |= (USART_CR1_RXNEIE);
}



void Process_Saved()
{
	for(int i=0;i<no_saved;i++)
	{

	for(int j=0;j<4;j++)
		{
		Input[j] = SavedInput[i][j];
		}
	Delay_ms(1000);
	ProcessInputCoded();
	}
	TransmitData("\nDo you want to repeat? (1 YES 2 NO) ");
	unsigned char temp=0;
	while(!(temp == 1 || temp == 2))
					{

					//Wait until data is received by polling
					while(!(USART1->SR & USART_SR_RXNE) );
					temp = USART1->DR;
					temp -= '0';
					//TransmitData("\nHello ");
					}
	if(temp==1)
	{
		Process_Saved();
	}
	else
	{
		Menu();
	}
}


unsigned char ProcessInputCoded()
{

	unsigned char rotation = (Input[1]-'0')*100 + (Input[2]-'0')*10 + (Input[3] - '0');
	if(rotation <=180 && rotation >= 0)
	{
		if(Input[0] == 'A')
				Servo_RotateA(rotation);
			else if(Input[0] == 'B')
				Servo_RotateB(rotation);
			else if(Input[0] == 'C')
				Servo_RotateC(rotation);
			else if(Input[0] == 'D')
				Servo_RotateD(rotation);
			else
				TransmitData("Error! ");
			return 1;
	}
	else
	{
		TransmitData("Error! Wrong rotation ");
		//USART1->CR1 &= ~(USART_CR1_RXNEIE);
		TransmitData(Input);
		return 0;
	}

}

void ProcessInputKeys()
{

}
