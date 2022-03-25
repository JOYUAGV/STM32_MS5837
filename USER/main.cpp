#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "ms5837.h"
#include "myiic.h"

MS5837 sensor;
int main(void)
{	 
	delay_init();	    	 //延时函数初始化	  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置中断优先级分组为组2：2位抢占优先级，2位响应优先级
	uart_init(115200);	 	//串口初始化为115200	 	
	IIC_Init();			//IIC初始化 
	while (!sensor.init()){
    printf("Init failed!");
    printf("Are SDA/SCL connected correctly?");
    printf("Blue Robotics Bar30: White=SDA, Green=SCL");
    printf("\n\n\n");
    delay_ms(1000);
		delay_ms(1000);
		delay_ms(1000);
  }
  //sensor.setModel(MS5837::MS5837_02BA);
	sensor.setModel(MS5837::MS5837_30BA);
  sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
  while (1)
  {
		sensor.read();
		printf("Depth: %.2f cm\t", sensor.depth()); 
		delay_ms(10);
		printf("Temprature: %.2f ℃\t", sensor.temperature());
		delay_ms(10);
		printf("Altitude: %.2f m\n", sensor.altitude()/100);  
		delay_ms(980);
		delay_ms(1000);
		delay_ms(1000);
  }
}
