#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <time.h>
#include <math.h>
#include <modbus/modbus.h>


// D6F, FAN1, SERVO1 connected
// SDP6, FAN2, SERVO2 connected
#define FAN1PIN "P9_14"
#define FAN2PIN "P9_16"
#define SERVO1PIN "P8_13"
#define SERVO2PIN "P8_19"
#define FAN1PWM "pwmchip5/pwm0"
#define FAN2PWM "pwmchip5/pwm1"
#define SERVO1PWM "pwmchip7/pwm0"
#define SERVO2PWM "pwmchip7/pwm1"
#define I2CDEV "/dev/i2c-2"
#define D6F "D6F-PH0505AD4"
#define SDP6 "SDP600-025PA"
#define D6F_ADDR 0x6C // D6F-PH0505AD4 address: 0x6C
#define SDP6_ADDR 0x40 // SDP600-025PA address: 0x40
#define PERIOD 10000000
#define CLOSE 1200000
#define OPEN 2400000
#define PERCISION 2
#define NB_BITS 16
#define NB_INPUT_BITS 16
#define NB_REGISTRERS 16
#define NB_INPUT_REGISTERS 16

modbus_t *ctx;
modbus_mapping_t *map;
// struct timespec ts; 
struct mb_tcp {
    uint16_t transact;
    uint16_t protocol;
    uint16_t length;
    uint8_t unit;
    uint8_t func;
    uint8_t data[];
};
/*
struct ahu_data {
    int dCycle1;
    int dCycle2;
    int dCycOpen1;
    int dCycOpen2;
    int presSet1;
    int presSet2;
    float flowRate1;
    float flowRate2; 
};
*/

int set_mb() {
    int i;
    const uint8_t UT_INPUT_BITS_TAB[] = {0xFF, 0xFF};
    ctx = modbus_new_tcp(NULL, 502);
    map = modbus_mapping_new_start_address(
        0, NB_BITS, 0, NB_INPUT_BITS, 0, NB_REGISTRERS, 16, NB_INPUT_REGISTERS);
    if(map == NULL) {
	printf("Unable to map registers\n");
	return 0;
    }else{
	printf("mapping, reg size %ld\n", sizeof(map->tab_input_registers));
    }
    modbus_set_bits_from_bytes(
	map->tab_input_bits, 0, NB_INPUT_REGISTERS, UT_INPUT_BITS_TAB);
    for(i = 0; i < NB_INPUT_REGISTERS; i++) {
	map->tab_input_registers[i] = 0;
//	printf("discrete input in register(%d) %d\n",i ,map->tab_input_bits[i]);
    }
    return i;
}

int tcp_respond(int* client, int regNum) {
    printf("connecting to MB...\n");
    int cl = modbus_tcp_listen(ctx, regNum);
    printf("awaiting the MB client...\n");
    modbus_tcp_accept(ctx, &cl);
    *client = cl;
    if(cl < 0) {
	printf("Unable to accept TCP\n");
	return 0;
    }
    return 1;
}

int mb_recieve(int* rc, uint8_t* buf) {
    printf("receiving MB data\n");
    *rc = modbus_receive(ctx,buf);
    if(rc < 0) {
	printf("Unable to recieve data by MB\n");
	return 0;
    }
    printf("recieved MB data\n");
    struct mb_tcp *frame = (struct mb_tcp *)buf;
    printf("function code %d\n", frame->func);
    printf("transaction %d\n",frame->transact);
//  for(int i=0;i<NB_REGISTRERS;i++){
//	printf("data in register(%d): %d\n", i, map->tab_registers[i]);
//  }
    printf("\n\n");
    return 1;
}

int mb_reply(int rc, uint8_t* buf) {
    printf("sending MB data\n");
    if(modbus_reply(ctx, buf, rc, map) < 0) {
	printf("Unable to send data by MB\n");
	return 0;
    }
    printf("sended MB data\n");
    struct mb_tcp *frame = (struct mb_tcp*)buf;
    printf("function code %d\n", frame->func);
    printf("transaction %d\n", frame->transact);
//  for(int i=0; i<NB_INPUT_REGISTERS; i++){
//	printf("data in register(%d): %d\n", i, map->tab_input_registers[i]);
//  }
    printf("\n\n");
    return 1;
}

int pwm_Init(char* pin){
    FILE *F = NULL;
    char add_pwm[64];
    sprintf(add_pwm, "/sys/devices/platform/ocp/ocp:%s_pinmux/state", pin);
    if ((F = fopen(add_pwm, "rb+")) == NULL){
        printf("Unable to open state handle\n");
        return 0;
    }
    fprintf(F, "%s", "pwm");
    fclose(F);
    return 1;
}

int set_period(char* pin, int per){
    FILE *F = NULL;
    char pwm_per[64];
    sprintf(pwm_per, "/sys/class/pwm/%s/period", pin);
    if ((F = fopen(pwm_per, "rb+")) == NULL){
        printf("Unable to open period handle\n");
        return 0;
    }
    fprintf(F, "%d", per);
    fclose(F);
    return 1;
}


int set_dCycle(char* pin, int dcyc){
    FILE *F = NULL;
    char pwm_dcyc[64];
    sprintf(pwm_dcyc, "/sys/class/pwm/%s/duty_cycle", pin);
    if ((F = fopen(pwm_dcyc, "rb+")) == NULL){
        printf("Unable to open period handle\n");
        return 0;
    }
    fprintf(F, "%d", dcyc);
    fclose(F);
    return 1;
}

int enable_pwm(char* pin, int status){
    FILE *F = NULL;
    char pwm_stat[64];
    sprintf(pwm_stat, "/sys/class/pwm/%s/enable", pin);
    if ((F = fopen(pwm_stat, "rb+")) == NULL){
        printf("Unable to open period handle\n");
        return 0;
    }
    fprintf(F, "%d", status);
    fclose(F);
    return 1;
}

uint16_t conv8us_u16_be(uint8_t* buf) {
    return (uint16_t)(((uint32_t)buf[0] << 8) | (uint32_t)buf[1]);
}

int16_t conv16us_s16(uint16_t buf) {
    return (int16_t)buf;
}

void delay(int msec) {
    struct timespec ts2 = {.tv_sec = msec / 1000,
                          .tv_nsec = (msec % 1000) * 1000000};
    nanosleep(&ts2, NULL);
}

/*
int delay2(long curTime, int del) {
    clock_gettime(CLOCK_REALTIME,&ts);
    if(labs(ts.tv_nsec - curTime) >= del) {
	return 1;
    }
    return 0;
}
*/

uint32_t i2c_write_reg16(uint8_t devAddr, uint16_t regAddr,
                         uint8_t* data , uint8_t length, char* sensor) {
    uint8_t buf[128];
    if (length > 127) {
        fprintf(stderr, "%s Byte write count (%d) > 127\n", sensor, length);
        return 11;
    }

    int fd = open(I2CDEV , O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "%s Failed to open device: %s\n", sensor, strerror(errno));
        return 12;
    }
    int err = 0;
    do {
        if (ioctl(fd, I2C_SLAVE, devAddr) < 0) {
            fprintf(stderr, "%s Failed to select device: %s\n", sensor, strerror(errno));
            err = 13; break;
        }
        buf[0] = regAddr >> 8;
        buf[1] = regAddr & 0xFF;
        if (length > 0) {
            memcpy(buf + 2, data, length);
        }
        length += 2;
        int count = write(fd, buf, length);
        if (count < 0) {
            fprintf(stderr, "%s Failed to write device(%d): %s\n",
                    sensor, count, strerror(errno));
            err = 14; break;
        } else if (count != length) {
            fprintf(stderr, "%s Short write to device, expected %d, got %d\n",
                    sensor, length, count);
            err = 15; break;
        }
    } while (false);
    close(fd);
    return err;
}

uint32_t i2c_read_reg8(uint8_t devAddr, uint8_t regAddr,
                       uint8_t* data, uint8_t length, char* sensor) {
    int fd = open(I2CDEV, O_RDWR);

    if (fd < 0) {
        fprintf(stderr, "%s Failed to open device: %s\n", sensor, strerror(errno));
        return 21;
    }
    int err = 0;
    do {
        if (ioctl(fd, I2C_SLAVE, devAddr) < 0) {
            fprintf(stderr, "%s Failed to select device: %s\n", sensor, strerror(errno));
            err = 22; break;
        }
        if (write(fd, &regAddr, 1) != 1) {
            fprintf(stderr, "%s Failed to write reg: %s\n", sensor, strerror(errno));
            err = 23; break;
        }
        int count = read(fd, data, length);
        if (count < 0) {
            fprintf(stderr, "%s Failed to read device(%d): %s\n",
                    sensor, count, strerror(errno));
            err = 24; break;
        } else if (count != length) {
            fprintf(stderr, "%s Short read  from device, expected %d, got %d\n",
                    sensor, length, count);
            err = 25; break;
        }
    } while (false);
    close(fd);
    return err;
}

int fan_control(char* fan, float flowRate, int dCycle, int press) {
    if (flowRate < press-PERCISION && dCycle < 100) {
        dCycle++;
    }
    else if (flowRate > press+PERCISION && dCycle > 0) {
        dCycle--;
    }
    set_dCycle(fan, PERIOD/100*dCycle);
    return dCycle;
}

void servo_control(char* servo, int dCycTurn) {
    set_dCycle(servo, (OPEN-CLOSE)/100*dCycTurn+CLOSE);
}

int main(void) {
    int dCycle1, newDCyc1, dCycle2, newDCyc2, 
	dCycOpen1, dCycOpen2, presSet1, presSet2; 
    int rc, regNum, client, temp, check, manual, end;
    uint8_t send0[] = {0x40, 0x18, 0x06};
    uint8_t send1[] = {0x51, 0x2C};
    uint8_t rD6F[2], rSDP6[2];
    uint8_t buf[MODBUS_TCP_MAX_ADU_LENGTH] = {};
    uint16_t rdFlow;
    float flowRate1, flowRate2;
    long timerSens, timerFans, timerMB;

    presSet2 = presSet1 = 15;	// setpoint for pessure
    dCycle1 = dCycle2 = 0;	// zero for fans
    dCycOpen1 = dCycOpen2 = 50;	// middle position for servo
    client = -1; 
    rc = 0;
    end = manual = check = 0;


    // 1. Initialize sensor D6F
    i2c_write_reg16(D6F_ADDR, 0x0B00, NULL, 0, D6F);
    delay(900);

    //2. Initialize PWM
    pwm_Init(FAN1PIN);
    pwm_Init(FAN2PIN);
    pwm_Init(SERVO1PIN);
    pwm_Init(SERVO2PIN);

    //3. Setup PWM
    set_period(FAN1PWM, PERIOD);
    set_dCycle(FAN1PWM, dCycle1);
    enable_pwm(FAN1PWM, 1);
    set_period(FAN2PWM, PERIOD);
    set_dCycle(FAN2PWM, dCycle2);
    enable_pwm(FAN2PWM, 1);
    set_period(SERVO1PWM, PERIOD);
    set_dCycle(SERVO1PWM, dCycOpen1);
    enable_pwm(SERVO1PWM, 1);
    set_period(SERVO2PWM, PERIOD);
    set_dCycle(SERVO2PWM, dCycOpen2);
    enable_pwm(SERVO2PWM, 1);

    // 5. Setup Modbus
    regNum = set_mb();

/*
    // 6. Timer setup
    clock_gettime(CLOCK_REALTIME,&ts);
    timerMB = timerFans = timerSens = ts.tv_nsec;
*/
    
    while(!end){

//	if(delay2(timerSens, 50000000)) {	// read sensors each 50msec
            
	    // 7.0 Trigger getting data D6F
            i2c_write_reg16(D6F_ADDR, 0x00D0, send0, 3, D6F);
	    delay(50);

	    // 7.1 Read data D6F
       	    i2c_write_reg16(D6F_ADDR, 0x00D0, send1, 2, D6F);
       	    i2c_read_reg8(D6F_ADDR, 0x07, rD6F, 2, D6F);
       	    rdFlow = conv8us_u16_be(rD6F);
   	    flowRate1 = fabsf(((float)rdFlow - 1024.0) * 100.0 / 60000.0 - 50.0);
       	    printf("%s %6.2f [Pa]\n", D6F, flowRate1);

            // 7.2 Read data SDP6
       	    i2c_read_reg8(SDP6_ADDR, 0xf1, rSDP6, 2, SDP6);
       	    rdFlow = conv8us_u16_be(rSDP6);
       	    flowRate2 = fabsf(((float)conv16us_s16(rdFlow))/1200);
       	    printf("%s %6.2f [Pa]\n", SDP6, flowRate2);

//	    timerSens = ts.tv_nsec;
//	}

//	if(delay2(timerFans, 250000000)){	// control fans each 250msec
            if(manual) {
		// 8.1 Manual control FAN1
		set_dCycle(FAN1PWM, PERIOD/100*dCycle1);
		printf("fan1 duty cycle %d\n", dCycle1);
		// 8.2 Manual control FAN2
		set_dCycle(FAN2PWM, PERIOD/100*dCycle2);
		printf("fan2 duty cycle %d\n", dCycle2);
	    } else {
		// 8.1 Control FAN1
		dCycle1 = fan_control(FAN1PWM, flowRate1, dCycle1, presSet1);
		printf("fan1 duty cycle %d\n", dCycle1); 

		// 8.2 Control FAN2
		dCycle2 = fan_control(FAN2PWM, flowRate2, dCycle2, presSet2);
		printf("fan2 duty cycle %d\n", dCycle2); 
	    }
//	    timerFans = ts.tv_nsec;
//	}

	// 9.1 Control SERVO1 
	servo_control(SERVO1PWM, dCycOpen1);
	printf("servo1 open %d\n", dCycOpen1); 

	// 9.2 Control SERVO2 
	servo_control(SERVO2PWM, dCycOpen2);
	printf("servo2 open %d\n", dCycOpen2);

	// 10. Establish MB connection
	if(regNum && !check) {
	    check = tcp_respond(&client, regNum); 
	    if(!check) close(client);
	}

//	if(delay2(timerMB, 500000000)){	// connect MB each 0.5sec
	    //11. Recieve data by MB
	    if(check) {
		if(mb_recieve(&rc, buf)) {
		    manual = map->tab_registers[6];
		    end = map->tab_registers[7];

		    dCycOpen1 = map->tab_registers[0];
		    dCycOpen2 = map->tab_registers[1];
		    if(manual) {
			dCycle1 = map->tab_registers[2];
			dCycle2 = map->tab_registers[3];
		    }
		    presSet1 = map->tab_registers[4];
		    presSet2 = map->tab_registers[5];  
		} else {
		    check = 0;
		    close(client);
		}
	    }

	    //12. Send data by MB
	    if(check) {
		map->tab_input_registers[0] = (uint16_t)dCycOpen1;
		map->tab_input_registers[1] = (uint16_t)dCycOpen2;
		map->tab_input_registers[2] = (uint16_t)dCycle1;
		map->tab_input_registers[3] = (uint16_t)dCycle2;
		map->tab_input_registers[4] = (uint16_t)flowRate1;
		map->tab_input_registers[5] = 
		    (uint16_t)((flowRate1 - map->tab_input_registers[4])*100);
		map->tab_input_registers[6] = (uint16_t)flowRate2;
		map->tab_input_registers[7] = 
		    (uint16_t)((flowRate2 - map->tab_input_registers[6])*100);
		if(!mb_reply(rc, buf)) {
		    check = 0;
	            close(client);
		}
	    }
//	    timerMB = ts.tv_nsec;
//	}
    }

    printf("\n ending programm\n");
    enable_pwm(FAN1PWM, 0);
    enable_pwm(FAN2PWM, 0);
    enable_pwm(SERVO1PWM, 0);
    enable_pwm(SERVO2PWM, 0);
    close(client);
    modbus_free(ctx);
    modbus_mapping_free(map);
    return 0;
}
