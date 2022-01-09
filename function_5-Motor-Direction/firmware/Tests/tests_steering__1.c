
/* Includes ------------------------------------------------------------------*/

#include "steering.h"
#include "stdlib.h"
#include "math.h"

#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <stdio.h>
#include "cmocka.h"


// declaration of ADCBUF (in main.c)
uint32_t ADCBUF[5];

// MOCK DEFINTIONS /////////////////////////////////////////////////////////////

// MOCK for HAL_GPIO_ReadPin
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  return mock();
}

//MOCK for HAL_GPIO_WritePin
void HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
  // empty mock
  return ;
}

//MOCK for HAL_Delay
void HAL_Delay(__IO uint32_t Delay)
{
  // empty mock
  return ;
}

void __wrap_steering_set_speed(GPIO_PinState en_steering, int speed){

  check_expected(speed);
  return ;
}

// TESTS DEFINITION ////////////////////////////////////////////////////////////

void test_steering_set_speed__1(void **state)
{
    (void) state; /* unused */
    int ret;

    expect_value(__wrap_steering_set_speed, speed, 10);
    steering_set_speed(0,0);

}



// TESTS EXECUTION /////////////////////////////////////////////////////////////


// test functions list
const struct CMUnitTest do_something_tests[] = {
    cmocka_unit_test(test_steering_set_speed__1),
};


// main function for tests
int main(void)
{
    return cmocka_run_group_tests(do_something_tests, NULL, NULL);
}


//void steering_set_speed(GPIO_PinState en_steering, int speed){
//    
//    /* Threshold rotating speed of steering wheels*/
//    if (speed > RIGHT_MAX_SPEED_STEERING){
//        speed = RIGHT_MAX_SPEED_STEERING;
//    } else if (speed < LEFT_MAX_SPEED_STEERING){
//        speed  = LEFT_MAX_SPEED_STEERING;
//    }
//    
//    speed = 3200 * ( speed/ 100.0 );
//    TIM1->CCR3 = speed;
//    
//    HAL_GPIO_WritePin( GPIOC, GPIO_PIN_12, en_steering);  //PC12  AV
//}

