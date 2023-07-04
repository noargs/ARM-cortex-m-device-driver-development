#include <stdint.h>
#include <string.h>

#include "ds1307.h"


static void ds1307_i2c_pin_config(void);
static void ds1307_i2c_config(void);
static void ds1307_write(uint8_t value, uint8_t reg_addr);
static uint8_t ds1307_read(uint8_t reg_addr);
static uint8_t bcd_to_binary(uint8_t value);
static uint8_t binary_to_bcd(uint8_t value);

I2C_Handle_t ds1307_i2c_handle;

// returns 1 : CH=1; init failed
// returns 0 : CH=0; init success
uint8_t ds1307_init(void)
{
  //1. initialise the i2c pins
  ds1307_i2c_pin_config();

  //2. initialise the i2c peripheral
  ds1307_i2c_config();

  //3. enable the I2C peripheral
  I2C_PeripheralControl(DS1307_I2C, ENABLE);

  //4. make clock halt = 0 which initially 1 i.e. CH=1
  //   [DS1307 Datasheet page: 8]
  ds1307_write(0x00, DS1307_ADDR_SEC);

  //5. read back clock halt bit i.e. CH
  uint8_t clock_state = ds1307_read(DS1307_ADDR_SEC);

  return ((clock_state >> 7) & 0x1);
}

void ds1307_set_current_time(RTC_time_t *rtc_time)
{
  uint8_t seconds, hours;
  seconds = binary_to_bcd(rtc_time->seconds);
  seconds &= ~(1 << 7);
  ds1307_write(seconds, DS1307_ADDR_SEC);

  ds1307_write(binary_to_bcd(rtc_time->minutes), DS1307_ADDR_MIN);

  hours = binary_to_bcd(rtc_time->hours);

  // DS3107 Datasheet page:8 [Table 2. Timekeeper Registers]
  // hours must be re-entered whenever BIT6 is changed
  if (rtc_time->time_format == TIME_FORMAT_24HRS){
	// BIT6=0 means 24HRS
	hours &= ~(1 << 6);
  }
  else
  {
	  // BIT6=1 means 12HRS
	hours |= (1 << 6);

    // BIT5=1 means PM, 0 means AM
	hours = (rtc_time->time_format == TIME_FORMAT_12HRS_PM) ? hours | (1 << 5) : hours & ~(1 << 5);
  }

  ds1307_write(hours, DS1307_ADDR_HRS);
}

void ds1307_set_current_date(RTC_date_t *rtc_date)
{
  ds1307_write(binary_to_bcd(rtc_date->date), DS1307_ADDR_DATE);
  ds1307_write(binary_to_bcd(rtc_date->month), DS1307_ADDR_MONTH);
  ds1307_write(binary_to_bcd(rtc_date->year), DS1307_ADDR_YEAR);
  ds1307_write(binary_to_bcd(rtc_date->day), DS1307_ADDR_DAY);
}

void ds1307_get_current_time(RTC_time_t *rtc_time)
{
  uint8_t seconds, hours;

  seconds = ds1307_read(DS1307_ADDR_SEC);
  seconds &= ~(1 << 7); // BIT7, CH is irrelevant
  rtc_time->seconds = bcd_to_binary(seconds);
  rtc_time->minutes = bcd_to_binary(ds1307_read(DS1307_ADDR_MIN));

  hours = ds1307_read(DS1307_ADDR_HRS);
  if (hours & (1 << 6))
  {
	//12hrs format
	// flip (!) so its equal to #define TIME_FORMAT_12HRS_AM  0
	//   or #define TIME_FORMAT_12HRS_PM  1
	rtc_time->time_format = !((hours & (1 << 5)) == 0);
	hours &= ~(0x3 << 5); //clear BIT6 and 5 as irrelevant
  }
  else
  {
	//24hrs format
	rtc_time->time_format = TIME_FORMAT_24HRS;
  }

  rtc_time->hours = bcd_to_binary(hours);

}

void ds1307_get_current_date(RTC_date_t *rtc_date)
{
  rtc_date->day = bcd_to_binary(ds1307_read(DS1307_ADDR_DAY));
  rtc_date->date = bcd_to_binary(ds1307_read(DS1307_ADDR_DATE));
  rtc_date->month = bcd_to_binary(ds1307_read(DS1307_ADDR_MONTH));
  rtc_date->year = bcd_to_binary(ds1307_read(DS1307_ADDR_YEAR));
}

static void ds1307_i2c_pin_config(void)
{
  GPIO_Handle_t i2c_sda, i2c_scl;

  memset(&i2c_sda,0,sizeof(i2c_sda));
  memset(&i2c_scl,0,sizeof(i2c_scl));

  // I2C1_SCL ==> PB6
  // I2C1_SDA ==> PB7

  i2c_sda.gpiox = DS1307_I2C_GPIO_PORT;
  i2c_sda.gpio_config.gpio_pin_alt_fun = 4;
  i2c_sda.gpio_config.gpio_pin_mode = GPIO_MODE_ALTFN;
  i2c_sda.gpio_config.gpio_pin_number = DS1307_I2C_SDA_PIN;
  i2c_sda.gpio_config.gpio_pin_op_type = GPIO_OP_TYPE_OD;
  i2c_sda.gpio_config.gpio_pin_pu_pd_control = DS1307_I2C_PUPD;
  i2c_sda.gpio_config.gpio_pin_speed = GPIO_SPEED_FAST;

  GPIO_Init(&i2c_sda);

  i2c_scl.gpiox = DS1307_I2C_GPIO_PORT;
  i2c_scl.gpio_config.gpio_pin_alt_fun = 4;
  i2c_scl.gpio_config.gpio_pin_mode = GPIO_MODE_ALTFN;
  i2c_scl.gpio_config.gpio_pin_number = DS1307_I2C_SCL_PIN;
  i2c_scl.gpio_config.gpio_pin_op_type = GPIO_OP_TYPE_OD;
  i2c_scl.gpio_config.gpio_pin_pu_pd_control = DS1307_I2C_PUPD;
  i2c_scl.gpio_config.gpio_pin_speed = GPIO_SPEED_FAST;

  GPIO_Init(&i2c_scl);
}

static void ds1307_i2c_config(void)
{
  ds1307_i2c_handle.i2cx = DS1307_I2C;
  ds1307_i2c_handle.i2c_config.i2c_ack_control = I2C_ACK_ENABLE;
  ds1307_i2c_handle.i2c_config.i2c_scl_speed = DS1307_I2C_SPEED;
  I2C_Init(&ds1307_i2c_handle);
}

static void ds1307_write(uint8_t value, uint8_t reg_addr)
{
  uint8_t tx[2];
  tx[0] = reg_addr;
  tx[1] = value;

  // Datasheet page:12 [Figure 4. Data Write - Slave Receiver Mode]
  I2C_MasterSendData(&ds1307_i2c_handle, tx, 2, DS1307_I2C_ADDRESS, I2C_DISABLE_SR);
}

static uint8_t ds1307_read(uint8_t reg_addr)
{
  uint8_t data;

  I2C_MasterSendData(&ds1307_i2c_handle, &reg_addr, 1, DS1307_I2C_ADDRESS, I2C_DISABLE_SR);
  I2C_MasterReceiveData(&ds1307_i2c_handle, &data, 1, DS1307_I2C_ADDRESS, I2C_DISABLE_SR);

  return data;
}

static uint8_t binary_to_bcd(uint8_t value)
{
  uint8_t m, n;
  uint8_t bcd;

  bcd = value;

  if (value >= 10)
  {
	m = value / 10;
	n = value % 10;
	bcd = (uint8_t)((m << 4) | n);
  }

  return bcd;
}

static uint8_t bcd_to_binary(uint8_t value)
{
  uint8_t m, n;
  m = (uint8_t)((value >> 4) * 10);
  n = value & (uint8_t)0x0F;
  return (m+n);
}







