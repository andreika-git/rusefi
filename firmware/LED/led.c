//=============================================================================
//�ļ�����:led.h
//���ܸ�Ҫ:STM32f401RBT6 ѭ������LED��
//��Ȩ����:Դ�ع�����http://vcc-gnd.taobao.com/
//�汾����:2015-02-20 V1.0
//���Է�ʽ:J-LINK-OB
//=============================================================================

//ͷ�ļ�
#include "led.h"

//=============================================================================
//��������:LED_GPIO_Config(void)
//���ܸ�Ҫ:LED����������
//��������:��
//��������:��
//=============================================================================
void LED_GPIO_Config(void)
{	
	//����һ��GPIO_InitTypeDef ���͵Ľṹ��
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOG,ENABLE);	//ʹ��GPIOC������ʱ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;	//ѡ��Ҫ�õ�GPIO����		 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��������Ϊ��ͨ���ģʽ		
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//��������Ϊ�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//���������ٶ�Ϊ100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//��������Ϊ���� 		 
	GPIO_Init(GPIOB, &GPIO_InitStructure);//���ÿ⺯������ʼ��GPIO
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;			 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 		 
	GPIO_Init(GPIOG, &GPIO_InitStructure);
}






