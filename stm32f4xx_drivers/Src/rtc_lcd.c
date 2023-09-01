#include <stdio.h>
#include "ds1307.h"

#include "lcd.h"

#define SYSTICK_TIM_CLK                  16000000UL

// ARM Cortex M4 Generic User Guide page: 249
void init_systick_timer(uint32_t tick_hz)
{
	uint32_t *systick_reload_value_register = (uint32_t*)0xE000E014;
	uint32_t *systick_control_status_register = (uint32_t*)0xE000E010;

	// calculate the reload value
	uint32_t count_value = (SYSTICK_TIM_CLK/tick_hz) - 1;

	// Clear the value of SRVR
	*systick_reload_value_register &= ~(0x00FFFFFF);

	// load the value in to SRVR
	*systick_reload_value_register |= count_value;

	// Enables SysTick exception request `TICKINT`
	// `1 = counting down to zero asserts the SysTick exception request`
	*systick_control_status_register |= (1 << 1);

	// Select clock source `0 = external clock` or `1 = processor clock` CLKSOURCE bit
	*systick_control_status_register |= (1 << 2);

	// enable the SysTick ENABLE bit `1 = counter enabled`
	*systick_control_status_register |= (1 << 0);
}


char* get_day_of_week(uint8_t i)
{
  char* days[] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
  return days[i-1];
}

void number_to_string(uint8_t num, char* buf)
{
  if (num < 10)
  {
	// if 4 ==> 04
	buf[0] = '0';
	buf[1] = num + 48; // add 48 to convert to ASCII
  }
  else if (num >= 10 && num < 99)
  {
	buf[0] = (num/10) + 48;
	buf[1] = (num % 10) + 48;
  }
}

//hh:mm:ss
char* time_to_string(RTC_time_t *rtc_time)
{
  static char buf[9];

  buf[2] = ':';
  buf[5] = ':';

  number_to_string(rtc_time->hours, buf);
  number_to_string(rtc_time->minutes, &buf[3]);
  number_to_string(rtc_time->seconds, &buf[6]);

  buf[8] = '\0';

  return buf;
}

// dd/mm/yy
char* date_to_string(RTC_date_t *rtc_date)
{
  static char buf[9];

  buf[2] = '/';
  buf[5] = '/';

  number_to_string(rtc_date->date, buf);
  number_to_string(rtc_date->month, &buf[3]);
  number_to_string(rtc_date->year, &buf[6]);

  buf[8] = '\0';

  return buf;
}

static void mdelay(uint32_t cnt)
{
	for(uint32_t i=0 ; i < (cnt * 1000); i++);
}

int main()
{
  RTC_time_t current_time;
  RTC_date_t current_date;

  //printf("RTC test\n");

	lcd_init();

  lcd_print_string("Hello Ibn");


	mdelay(2000);

	lcd_display_clear();
	lcd_display_return_home();

  lcd_print_string("Hello Ibn");





  if (ds1307_init())
  {
  	printf("RTC init has failed\n");
  	while(1);
  }

  // 1 sec, give value 10 for 10 sec
  init_systick_timer(1);

  current_date.day = MONDAY;
  current_date.date = 18;
  current_date.month = 7;
  current_date.year = 23; // Datasheet page: 8, range 00-99

  current_time.hours = 1;
  current_time.minutes = 38;
  current_time.seconds = 41;
  current_time.time_format = TIME_FORMAT_12HRS_AM;

  ds1307_set_current_date(&current_date);
  ds1307_set_current_time(&current_time);

  ds1307_get_current_date(&current_date);
  ds1307_get_current_time(&current_time);

  char *am_pm;
  if (current_time.time_format != TIME_FORMAT_24HRS)
  {
    am_pm = (current_time.time_format) ? "PM" : "AM";
    printf("Current time = %s %s\n", time_to_string(&current_time), am_pm); // 01:38:41 AM
  }
  else
  {
	printf("Current time = %s\n", time_to_string(&current_time)); // 01:38:41
  }

  // 4/7/23 <Tuesday>
  printf("Current date = %s <%s>\n", date_to_string(&current_date), get_day_of_week(current_date.day));

  while(1);

  return 0;
}

void SysTick_Handler (void)
{
  RTC_time_t current_time;
  RTC_date_t current_date;

  ds1307_get_current_time(&current_time);

  char *am_pm;
  if (current_time.time_format != TIME_FORMAT_24HRS)
  {
    am_pm = (current_time.time_format) ? "PM" : "AM";
    printf("Current time = %s %s\n", time_to_string(&current_time), am_pm); // 01:38:41 AM
  }
	else
  {
  	printf("Current time = %s\n", time_to_string(&current_time)); // 01:38:41
  }

  ds1307_get_current_date(&current_date);
  // 4/7/23 <Tuesday>
  printf("Current date = %s <%s>\n", date_to_string(&current_date), get_day_of_week(current_date.day));

}
