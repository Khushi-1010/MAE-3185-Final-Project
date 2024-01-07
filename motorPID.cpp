#include <stdio.h>
#include <stdlib.h>
#include <pico/stdlib.h>
#include <hardware/gpio.h>
#include <hardware/timer.h>
#include <hardware/pwm.h>
#include <hardware/i2c.h>
#include<hardware/uart.h>

#define PWMS_MOT 6
#define GPIO_MOT_IN1 12
#define GPIO_MOT_IN2 13

#define PWMS_CONTROL 0
#define GPIO_SDA_TS 16
#define GPIO_SCL_TS 17
#define GPIO_A 3
#define GPIO_B 4

// Define the I2C address of the Adafruit TSC2007
#define TSC2007_ADDR 0b1001000
#define X_CMD 0b11000000
#define Y_CMD 0b11010000
uint16_t x_raw=0, y_raw=0;
int des_x=2047, des_y=2047; //mid 2047


uint16_t readcoordinates()
{
    uint8_t touchDataX[2]; //2 bytes for x and 2 bytes for y
    uint8_t touchDataY[2];
    uint8_t X_INFO[1] = {X_CMD};
    uint8_t Y_INFO[1]={Y_CMD};
    // Read touch data from the TSC2007
    i2c_write_blocking(i2c0, TSC2007_ADDR, X_INFO, 1, true); 
    i2c_read_blocking(i2c0, TSC2007_ADDR, touchDataX, 2, false); 
    i2c_write_blocking(i2c0, TSC2007_ADDR, Y_INFO, 1, true);
    i2c_read_blocking(i2c0, TSC2007_ADDR, touchDataY, 2, false); 
    
    x_raw = ((uint16_t)touchDataX[0]<<4) | ((uint16_t)touchDataX[1]>>4);
    y_raw = ((uint16_t)touchDataY[0]<<4) | ((uint16_t)touchDataY[1]>>4);
    return x_raw, y_raw;
}

void controlMotors()
{
    float kp_x = (750 / 4095.0) , kd_x = 0.07, ki_x = 0.1;

//FOR X COORDINATE

    static float eLast_x = 0;
    static float ei_x = 0;

    float e_x = float(des_x) - (x_raw); //e_x is error for x location
    float ed_x = (e_x - eLast_x) / 20e-3;
    ei_x = ei_x + e_x * 20e-3;

//FOR Y-COORDINATE
    float kp_y = (750 / 4095.0), kd_y = 0.07, ki_y = 0.1;

    static float eLast_y = 0;
    static float ei_y = 0;

    float e_y = float(des_y) - (y_raw); //e_y is error for y location
    float ed_y = (e_y - eLast_y) / 20e-3;
    ei_y = ei_y + e_y * 20e-3;
    //printf("e_x: %f\t e_y: %f\n",e_x,e_y);
    //printf("EI_X: %f\t EI_y: %f\n",ei_x,ei_y);
    if (ei_x < 750)
        ei_x = 750;
    else if (ei_x > 3000)
        ei_x = 3000;


     if (ei_y < 750)
        ei_y = 750;
    else if (ei_y > 3000)
        ei_y = 3000;

    float tau_x = kp_x * e_x + kd_x * ed_x + ki_x * ei_x;
    float tau_y = kp_y * e_y + kd_y * ed_y + ki_y * ei_y;
    
    float cc_x=0,cc_y=0;
    cc_x=(0.865*tau_x)+1875;
    cc_y=(1.023*tau_y)+1875;
    if (cc_x>2375)//limits the movement of motor till 130 degrees because of design constraint
        cc_x=2375;

    if (cc_y>2375)
        cc_y=2375;

    pwm_set_chan_level(PWMS_MOT,0,cc_y);
    pwm_set_chan_level(PWMS_MOT,1,cc_x);

   
    eLast_x = e_x;
    eLast_y = e_y;
}

void handleGpioIRQ(uint gpio, uint32_t event)
{
    if (gpio == GPIO_A && event== GPIO_IRQ_EDGE_RISE)
        readcoordinates();
    if (gpio == GPIO_B && event== GPIO_IRQ_EDGE_RISE)
        if (x_raw!=0 && y_raw!= 4095)
            controlMotors();
        else
            {pwm_set_chan_level(PWMS_MOT, 1, 1875);
            pwm_set_chan_level(PWMS_MOT, 0, 1875);}
    
}
void RChars()
{
    while (uart_is_readable_within_us(uart0, 1000))
        printf("%c", uart_getc(uart0));
        sleep_ms(1000);
}
void setup()
{
    stdio_init_all();
    sleep_ms(10000);
    gpio_init(GPIO_SDA_TS);
    gpio_init(GPIO_SCL_TS);
    gpio_set_function(GPIO_SDA_TS,GPIO_FUNC_I2C);
    gpio_set_function(GPIO_SCL_TS,GPIO_FUNC_I2C);

    gpio_set_pulls(GPIO_SDA_TS,1,0); // Pull-up resistor for SDA
    gpio_set_pulls(GPIO_SCL_TS,1,0); // Pull-up resistor for SCL
    i2c_init(i2c0, 100000); // Initialize I2C0 with a speed of 100 kHz

    gpio_init(GPIO_MOT_IN1);
    gpio_set_dir(GPIO_MOT_IN1, true);
    gpio_set_function(GPIO_MOT_IN1, GPIO_FUNC_PWM);

    gpio_init(GPIO_MOT_IN2);
    gpio_set_dir(GPIO_MOT_IN2, true);
    gpio_set_function(GPIO_MOT_IN2, GPIO_FUNC_PWM);

    gpio_init(GPIO_A);
    gpio_set_dir(GPIO_A, true);
    gpio_set_function(GPIO_A, GPIO_FUNC_PWM);

    pwm_set_clkdiv_int_frac(1, 100, 0);//WORKS FOR 5MS
    pwm_set_wrap(1, 6249);
    pwm_set_chan_level(1, 1, 3125);
    pwm_set_enabled(1, 1);

    gpio_init(GPIO_B);
    gpio_set_dir(GPIO_B, true);
    gpio_set_function(GPIO_B, GPIO_FUNC_PWM);

    pwm_set_clkdiv_int_frac(2, 100, 0);//WORKS FOR 20MS
    pwm_set_wrap(2, 24999);
    pwm_set_chan_level(2, 0, 12500);
    pwm_set_enabled(2, 1);

    gpio_set_irq_callback(handleGpioIRQ);
    gpio_set_irq_enabled(GPIO_A, GPIO_IRQ_EDGE_RISE, 1);
    gpio_set_irq_enabled(GPIO_B, GPIO_IRQ_EDGE_RISE, 1);
    irq_set_enabled(IO_IRQ_BANK0, 1);

    // calls motors function every 20ms
    pwm_set_clkdiv_int_frac(PWMS_MOT, 100, 0);
    pwm_set_wrap(PWMS_MOT, 24999);//top value
    pwm_set_chan_level(PWMS_MOT, 0, 1875);
    pwm_set_enabled(PWMS_MOT, true);

    pwm_set_clkdiv_int_frac(PWMS_MOT, 100, 0);
    pwm_set_wrap(PWMS_MOT, 24999);//top value
    pwm_set_chan_level(PWMS_MOT, 1, 1875);
    pwm_set_enabled(PWMS_MOT, true);
    
    //initializing the bluetooth module

    gpio_init(0);
    gpio_init(1);
    gpio_set_function(0,GPIO_FUNC_UART);
    gpio_set_function(1,GPIO_FUNC_UART);
    uart_init(uart0,38400);
    sleep_ms(10000);
    //uart_puts(uart0,"AT\r\n");
/*
    for(int i=0; i < 10; ++i)
    {
        uart_puts(uart0, "AT\r\n");
        RChars();
        sleep_ms(1000);
    }
    //printf("Setting Default values\r\n");
    //sleep_ms(1000);
    //uart_puts(uart0, "AT+ORGL\r\n");
    //RChars(); 

    printf("Setting name\r\n");
    uart_puts(uart0, "AT+NAME=VatsalRocks\r\n");
    RChars();

    printf("Password\r\n");
    uart_puts(uart0, "AT+PSWD=1234\r\n");
    RChars();

    printf("UART\r\n");
    uart_puts(uart0, "AT+UART=38400,0,0\r\n");
    RChars(); */

}
void loop()
{
    //printf("X_raw: %u\tY_raw: %u\n",x_raw,y_raw);
    char c[4]={0};
    char d[4]={0};
    char var=uart_getc(uart0);
    if (var == '\r')
    {
        uart_getc(uart0);
        c[0]= uart_getc(uart0);
        c[1]= uart_getc(uart0);
        c[2]= uart_getc(uart0);
        c[3]= uart_getc(uart0);

        des_x = atoi(c);
        printf("%d\t",des_x);
    }
    else if (var == ',')
    {
        d[0]= uart_getc(uart0);
        d[1]= uart_getc(uart0);
        d[2]= uart_getc(uart0);
        d[3]= uart_getc(uart0);

        des_y = atoi(d);
        printf("%d\n",des_y);
    }
}
int main()
{
    setup();

    while (true)
        loop();
        
}