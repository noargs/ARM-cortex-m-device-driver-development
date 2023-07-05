#include <stdio.h>
#include "ds1307.h"

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


int main()
{
  RTC_time_t current_time;
  RTC_date_t current_date;

  printf("RTC test\n");

  if (ds1307_init())
  {
	printf("RTC init has failed\n");
	while(1);
  }

  current_date.day = TUESDAY;
  current_date.date = 4;
  current_date.month = 7;
  current_date.year = 23; // Datasheet page: 8, range 00-99

  current_time.hours = 7;
  current_time.minutes = 38;
  current_time.minutes = 41;
  current_time.time_format = TIME_FORMAT_12HRS_PM;

  ds1307_set_current_date(&current_date);
  ds1307_set_current_time(&current_time);

  ds1307_get_current_date(&current_date);
  ds1307_get_current_time(&current_time);

  char *am_pm;
  if (current_time.time_format != TIME_FORMAT_24HRS)
  {
    am_pm = (current_time.time_format) ? "PM" : "AM";
    printf("Current time = %s %s\n", time_to_string(&current_time), am_pm); // 07:38:41 PM
  }
  else
  {
	printf("Current time = %s\n", time_to_string(&current_time)); // 19:38:41
  }

  // 4/7/23 <Tuesday>
  printf("Current date = %s <%s>\n", date_to_string(&current_date), get_day_of_week(current_date.day));

  return 0;
}
