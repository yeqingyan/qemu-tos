/*
 * Raspberry Pi emulation (c) 2012 Gregory Estrade
 * This code is licensed under the GNU GPLv2 and later.
 */

#include "hw/sysbus.h"
#include <sys/types.h>         
#include <sys/socket.h>
#include <string.h>
#include <netdb.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <json/json.h>

#define TYPE_BCM2835_TODO "bcm2835_todo"
#define BCM2835_TODO(obj) \
        OBJECT_CHECK(bcm2835_todo_state, (obj), TYPE_BCM2835_TODO)

#define GPIO_MEM_SIZE 0xA0
/*
// GPIO Registers offset
#define GPSEL0 0x0
#define GPSEL1 0x4
#define GPSEL2 0x8
#define GPSEL3 0xC
#define GPSEL4 0x10
#define GPSEL5 0x14
#define GPSET0  0x1C
#define GPSET1  0x2D
#define GPCLR0  0x28
#define GPCLR1  0x2C 
#define GPLEV0  0x34
#define GPLEV1  0x38

#define GPEDS0  0x40 
#define GPEDS1  0x44 
#define GPREN0  0x4c 
#define GPREN1  0x50 
#define GPFEN0  0x58 
#define GPFEN1  0x5c 
#define GPHEN0  0x64 
#define GPHEN1  0x68 
#define GPLEN0  0x70 
#define GPLEN1  0x74 
#define GPAREN0  0x7c 
#define GPAREN1  0x80 
#define GPAFEN0  0x88 
#define GPAFEN1  0x8c 

#define GPPUD 0x94 

#define GPPUDCLK0   0x98 
#define GPPUDCLK1   0x9c 
*/


//#define check_bit(var,pos)  ((var) & (1<<(pos)))

typedef struct {
    SysBusDevice busdev;
    MemoryRegion iomem;
    int socketfd;

    uint32_t GPFSEL0;   /* 0x00 Function Select Pins 0-9   */
    uint32_t GPFSEL1;   /* 0x04 Function Select Pins 10-19 */
    uint32_t GPFSEL2;   /* 0x08 Function Select Pins 20-29 */
    uint32_t GPFSEL3;   /* 0x0c Function Select Pins 30-39 */
    uint32_t GPFSEL4;   /* 0x10 Function Select Pins 40-49 */
    uint32_t GPFSEL5;   /* 0x14 Function Select Pins 50-53 */

    uint32_t GPSET0;    /* 0x1c Output Set Pins  0-31 [0=No Effect | 1=Set] */
    uint32_t GPSET1;    /* 0x20 Output Set Pins 32-53 [0=No Effect | 1=Set] */

    uint32_t GPCLR0;    /* 0x28 Output Clear Pins   0-31 [0=No Effect | 1=Clear] */
    uint32_t GPCLR1;    /* 0x2c Output Clear Pins  32-53 [0=No Effect | 1=Clear] */

    uint32_t GPLEV0;    /* 0x34 Pin Level (Read Only)  0-31 [0=Low | 1=High] */
    uint32_t GPLEV1;    /* 0x38 Pin Level (Read Only) 32-53 [0=Low | 1=High] */

    uint32_t GPEDS0;    /* 0x40 Event Detect Pins  0-31 [0=Event Not Detected | 1=Event Detected] */
    uint32_t GPEDS1;    /* 0x44 Event Detect Pins 32-53 [0=Event Not Detected | 1=Event Detected] */

    uint32_t GPREN0;    /* 0x4c Rising Edge Detect Enable Pins  0-31 [0=Rising Edge Detect Disabled | 1=Rising Edge Detect Enabled] */
    uint32_t GPREN1;    /* 0x50 Rising Edge Detect Enable Pins 32-53 [0=Rising Edge Detect Disabled | 1=Rising Edge Detect Enabled] */

    uint32_t GPFEN0;    /* 0x58 Falling Edge Detect Enable Pins  0-31 [0=Falling Edge Detect Disabled | 1=Falling Edge Detect Enabled] */
    uint32_t GPFEN1;    /* 0x5c Falling Edge Detect Enable Pins 32-53 [0=Falling Edge Detect Disabled | 1=Falling Edge Detect Enabled] */

    uint32_t GPHEN0;    /* 0x64 High Level Detect Enable Bits  0-31 [0=High Level Detect Disabled | 1=High Level Detect Enabled] */
    uint32_t GPHEN1;    /* 0x68 High Level Detect Enable Bits 32-53 [0=High Level Detect Disabled | 1=High Level Detect Enabled] */
   
    uint32_t GPLEN0;    /* 0x70 Low Level Detect Enable Bits  0-31 [0=Low Level Detect Disabled | 1=Low Level Detect Enabled] */
    uint32_t GPLEN1;    /* 0x74 Low Level Detect Enable Bits 32-53 [0=Low Level Detect Disabled | 1=Low Level Detect Enabled] */
    
    uint32_t GPAREN0;   /* 0x7c Async Rising Edge Detect Enable Pins  0-31 [1=Async Rising Edge Detect Disabled | 1=Async Rising Edge Detect Enabled] */
    uint32_t GPAREN1;   /* 0x80 Async Rising Edge Detect Enable Pins 32-53 [1=Async Rising Edge Detect Disabled | 1=Async Rising Edge Detect Enabled] */
    
    uint32_t GPAFEN0;   /* 0x88 Async Falling Edge Detect Enable Pins  0-31 [1=Async Falling Edge Detect Disabled | 1=Async Falling Edge Detect Enabled] */
    uint32_t GPAFEN1;   /* 0x8c Async Falling Edge Detect Enable Pins 32-53 [1=Async Falling Edge Detect Disabled | 1=Async Falling Edge Detect Enabled] */
    
    uint32_t GPPUD;     /* 0x94 Pull-Up/Pull-Down All Pins [Bits 1-0: 0=Disable Pull-Up/Pull-Down | 1=Enable Pull-Down Control | 2=Enable Pull-Up Control] */
    uint32_t GPPUDCLK0; /* 0x98 Pull-Up/Pull-Down Clock Pins  0-31 [0=No Effect | 1=Assert Clock on Line] */
    uint32_t GPPUDCLK1; /* 0x9c Pull-Up/Pull-Down Clock Pins 32-53 [0=No Effect | 1=Assert Clock on Line] */

    uint32_t PINSET0; /* Derived output state for pins  0-31 based on SET and CLR registers */
    uint32_t PINSET1; /* Derived output state for pins  32-53 based on SET and CLR registers */

    uint32_t PUDSET0; /*Derived state for pull up or pull down for Pins  0-15 [0b00= No PUD | 0b01 = Pull Up | 0b10 = Pull Down | 0b11 = Reserved ] */
    uint32_t PUDSET1; /*Derived state for pull up or pull down for Pins 16-31 [0b00= No PUD | 0b01 = Pull Up | 0b10 = Pull Down | 0b11 = Reserved ] */
    uint32_t PUDSET2; /*Derived state for pull up or pull down for Pins 32-47 [0b00= No PUD | 0b01 = Pull Up | 0b10 = Pull Down | 0b11 = Reserved ] */
    uint32_t PUDSET3; /*Derived state for pull up or pull down for Pins 48-53 [0b00= No PUD | 0b01 = Pull Up | 0b10 = Pull Down | 0b11 = Reserved ] */


} bcm2835_todo_state;

/* Get pin number */
static int get_pin(uint64_t value) {
    int pin = 0;
    while ((pin < 32) && ((value & 1) == 0)) {
        value = value >> 1;
        pin += 1;
    }
    
    if(pin >=32) {
        fprintf(stderr, "[QEMU][Raspi] Unknown pin number %d!", pin);
        exit(1);
    }
    return pin;
}


/**
 * Return pin function
 * @param s     Device State
 * @param pin   Pin Number
 * @return      current pin function selection
 */

static uint32_t get_function (bcm2835_todo_state *s, int pin){

  /* GPFSELx registers have 3 bits per pin, 10 pins per register */
  if (pin<10)            return ((s->GPFSEL0 >> (3*pin))      & 0x7);
  if (pin>=10 && pin<20) return ((s->GPFSEL1 >> (3*(pin-10))) & 0x7);
  if (pin>=20 && pin<30) return ((s->GPFSEL2 >> (3*(pin-20))) & 0x7);
  if (pin>=30 && pin<40) return ((s->GPFSEL3 >> (3*(pin-30))) & 0x7);
  if (pin>=40 && pin<50) return ((s->GPFSEL4 >> (3*(pin-40))) & 0x7);
  return ((s->GPFSEL5 >> (3*(pin-50))) & 0x7);

}

static int setup_socket(void) {
    const int SERVER_PORT = 8988;
    struct sockaddr_in serv_addr;
    struct hostent *local;
    int socketfd;
    fprintf(stderr, "start setup socket\n");

    local = gethostbyname("localhost");
    if (local == NULL) {
        fprintf(stderr, "[QEMU][Raspi] Can not get hostname");
        exit(1);
    }
    socketfd = 0;
    socketfd = socket(AF_INET, SOCK_STREAM, 0);

    if (socketfd < 0) {
        fprintf(stderr, "[QEMU][Raspi] ERROR opening socket");
        exit(1);
    }

    memset((char *)&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)local->h_addr, (char *)&serv_addr.sin_addr.s_addr, local->h_length);
    serv_addr.sin_port = htons(SERVER_PORT);

    if (connect(socketfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0){
        fprintf(stderr, "[QEMU][Raspi] ERROR connecting(%d): %s \n", errno, strerror(errno));
        exit(1);
    }
    return socketfd;
}

static char * convert_to_string(uint32_t reg)
{
	char buffer[30] = {0};
	sprintf(buffer,"%.9d", reg);
	printf("%s\n", buffer);
	return buffer; 
}

static uint64_t read_gpio(bcm2835_todo_state *s) {

    int i, mask;
    char buf[256];
    char buffer[50];
    //unsigned int value = 0;
    int n;
    int GPLEV0=0,GPLEV1=0;
    bzero(buf, 256);  //places 256 null bytes in the string buf

        char GPFSEL0[30] = {0};
	char GPFSEL1[30] = {0};
	char GPFSEL2[30] = {0};
	char GPFSEL3[30] = {0};
	char GPFSEL4[30] = {0};
	char GPFSEL5[30] = {0};
	char PINSET0[30] = {0};
	char PINSET1[30] = {0};
	
	sprintf(GPFSEL0,"%d", s->GPFSEL0);
	sprintf(GPFSEL1,"%d", s->GPFSEL1);
	sprintf(GPFSEL2,"%d", s->GPFSEL2);
	sprintf(GPFSEL3,"%d", s->GPFSEL3);
	sprintf(GPFSEL4,"%d", s->GPFSEL4);
	sprintf(GPFSEL5,"%d", s->GPFSEL5);
	sprintf(PINSET0,"%d", s->PINSET0);
	sprintf(PINSET1,"%d", s->PINSET1);

	json_object *jobj = json_object_new_object();
	json_object_object_add(jobj,"GPFSEL0", json_object_new_string(GPFSEL0));
	json_object_object_add(jobj,"GPFSEL1", json_object_new_string(GPFSEL1));
	json_object_object_add(jobj,"GPFSEL2", json_object_new_string(GPFSEL2));
	json_object_object_add(jobj,"GPFSEL3", json_object_new_string(GPFSEL3));
	json_object_object_add(jobj,"GPFSEL4", json_object_new_string(GPFSEL4));
	json_object_object_add(jobj,"GPFSEL5", json_object_new_string(GPFSEL5));
	json_object_object_add(jobj,"PINSET0", json_object_new_string(PINSET0));
	json_object_object_add(jobj,"PINSET1", json_object_new_string(PINSET1));
	json_object_object_add(jobj,"READ", json_object_new_boolean(true));  //value is true so that simulator returns pinset0 and pinset1 values

	const char * string = json_object_to_json_string(jobj);
	printf("\n read %s  size: %d\n", string,strlen(string));
   	n=write(s->socketfd, string, strlen(string));
	n=write(s->socketfd, "\n", 1);
	if (n < 0) 
	{
        fprintf(stderr, "[QEMU][Raspi] ERROR %s writing to socket\n", strerror(errno));
        exit(1);
   	} 

    
    /* read 255 bytes from the socket into the buffer pointed to by buf */
    while(read(s->socketfd, buf, 255) > 0); // buf will have json string
   /* if (n < 0) {
        fprintf(stderr, "[QEMU][Raspi] READ ERROR %s, %s\n", strerror(errno), buf);
        exit(1);
    }*/
    
	printf("String received: %s\n\n", buf);
    json_object * obj = json_tokener_parse(buf);  
	GPLEV0 =json_object_get_int(obj);
	GPLEV1 =json_object_get_int(obj);

    // fprintf(stderr, "\n[QEMU] TOS READ GPIO: read %d from %x\n", value, (unsigned int)offset);
	for (i=0; i<32; i++)
	{
	    mask = (1 << i);
	    if (get_function(s,i) == 0x00){ /* only check input pins */
	      if ((mask & GPLEV0) > 0) s->GPLEV0 |= mask;
	      else s->GPLEV0 &= ~mask;
	    }
    	}
	for (i=32; i<54; i++)
	{
	    mask = (1 << i);
	    if (get_function(s,i) == 0x00){ /* only check input pins */
	      if ((mask & GPLEV1) > 0) s->GPLEV1 |= mask;
	      else s->GPLEV1 &= ~mask;
	    }
	}
}




static void write_gpio(bcm2835_todo_state *s)
{
    int i;
    //char buf[256];
    int n, pin; 
    uint32_t mask;
   // bzero(buf, 256);

    for (i=0; i<32; i++)
    {
        mask = (1 << i);
        if (get_function(s,i) == 0x01){    /* making sure its an output */
          if (s->GPSET0 & mask){          /* set flag is set */
            s->PINSET0 |= mask;            /* set the state bit */
            s->GPSET0 &= ~mask;           /* clear the set bit */
          }
          else if (s->GPCLR0 & mask){     /* clear flag is set */
            s->PINSET0 &= ~mask;            /* set the state bit */
            s->GPCLR0 &= ~mask;           /* clear the set bit */
            }
         }
    }
    for (i=32; i<54; i++)
    {
        mask = (1 << (i-32));
        if (get_function(s,i) == 0x01){    /* making sure its an output */
          if (s->GPSET1 & mask){          /* set flag is set */
            s->PINSET1 |= mask;            /* set the state bit */
            s->GPSET1 &= ~mask;           /* clear the set bit */
          }
          else if (s->GPCLR1 & mask){     /* clear flag is set */
            s->PINSET1 &= ~mask;            /* set the state bit */
            s->GPCLR1 &= ~mask;           /* clear the set bit */
          }
        }
     }


/*
   fprintf(stderr, "[QEMU][Raspi] \n*******************\n");
   fprintf(stderr, "[QEMU][Raspi] PINSET0  = %x!\n", s->PINSET0);
   fprintf(stderr, "[QEMU][Raspi] PINSET1  = %x!\n", s->PINSET1);
   fprintf(stderr, "[QEMU][Raspi] GPFSEL0  = %x!\n", s->GPFSEL0);
   fprintf(stderr, "[QEMU][Raspi] GPFSEL1  = %x!\n", s->GPFSEL1);
   fprintf(stderr, "[QEMU][Raspi] GPFSEL2  = %x!\n", s->GPFSEL2);
   fprintf(stderr, "[QEMU][Raspi] GPFSEL3  = %x!\n", s->GPFSEL3);
   fprintf(stderr, "[QEMU][Raspi] GPFSEL4  = %x!\n", s->GPFSEL4);
   fprintf(stderr, "[QEMU][Raspi] GPFSEL5  = %x!\n", s->GPFSEL5);*/
	



	char GPFSEL0[30] = {0};
	char GPFSEL1[30] = {0};
	char GPFSEL2[30] = {0};
	char GPFSEL3[30] = {0};
	char GPFSEL4[30] = {0};
	char GPFSEL5[30] = {0};
	char PINSET0[30] = {0};
	char PINSET1[30] = {0};
	
	sprintf(GPFSEL0,"%d", s->GPFSEL0);
	sprintf(GPFSEL1,"%d", s->GPFSEL1);
	sprintf(GPFSEL2,"%d", s->GPFSEL2);
	sprintf(GPFSEL3,"%d", s->GPFSEL3);
	sprintf(GPFSEL4,"%d", s->GPFSEL4);
	sprintf(GPFSEL5,"%d", s->GPFSEL5);
	sprintf(PINSET0,"%d", s->PINSET0);
	sprintf(PINSET1,"%d", s->PINSET1);


	/*Creating a json object to send to GPIO Simulator*/
	json_object *jobj = json_object_new_object();
	//convert_to_string(s->GPFSEL0);
	 //fprintf(stderr, "[QEMU][Raspi] im here");
	json_object_object_add(jobj,"GPFSEL0", json_object_new_string(GPFSEL0));
	json_object_object_add(jobj,"GPFSEL1", json_object_new_string(GPFSEL1));
	json_object_object_add(jobj,"GPFSEL2", json_object_new_string(GPFSEL2));
	json_object_object_add(jobj,"GPFSEL3", json_object_new_string(GPFSEL3));
	json_object_object_add(jobj,"GPFSEL4", json_object_new_string(GPFSEL4));
	json_object_object_add(jobj,"GPFSEL5", json_object_new_string(GPFSEL5));
	json_object_object_add(jobj,"PINSET0", json_object_new_string(PINSET0));
	json_object_object_add(jobj,"PINSET1", json_object_new_string(PINSET1));
	json_object_object_add(jobj,"READ", json_object_new_boolean(false)); // this field is used to by Simulator to decide whether to send back PINSET values or not

	const char * string = json_object_to_json_string(jobj);
	printf("\nwrite %s  size: %d\n", string,strlen(string));
        //fprintf(stderr, "[QEMU][Raspi] String %s writing to socket\n", string);
	n = write(s->socketfd, string, strlen(string)); // Send the JSON String to GPIO Simulator
   	n = write(s->socketfd, "\n", 1);
    if (n < 0) {
        fprintf(stderr, "[QEMU][Raspi] ERROR %s writing to socket\n", strerror(errno));
        exit(1);
    }    

    return;
}


/* This fuction is called upon read detection */
/* Returns the device state field corresponding to the read address */

static uint64_t bcm2835_todo_read(void *opaque, hwaddr offset,unsigned size)
{
    uint64_t value;
    bcm2835_todo_state *s = (bcm2835_todo_state *)opaque;
   
   switch (offset) {
	case 0x00: return s->GPFSEL0;
	case 0x04: return s->GPFSEL1;
	case 0x08: return s->GPFSEL2;
	case 0x0c: return s->GPFSEL3;
	case 0x10: return s->GPFSEL4;
	case 0x14: return s->GPFSEL5;
	case 0x34: read_gpio(s);//read from gpio simulator
			return s->GPLEV0;
	case 0x38: read_gpio(s);//read from gpio simulator
			return s->GPLEV1;
	case 0x40: return s->GPEDS0;
	case 0x44: return s->GPEDS1;
	case 0x4c: return s->GPREN0;
	case 0x50: return s->GPREN1;
	case 0x58: return s->GPFEN0;
	case 0x5c: return s->GPFEN1;
	case 0x64: return s->GPHEN0;
	case 0x68: return s->GPHEN1;
	case 0x70: return s->GPLEN0;
	case 0x74: return s->GPLEN1;
	case 0x7c: return s->GPAREN0;
	case 0x80: return s->GPAREN1;
	case 0x88: return s->GPAFEN0;
	case 0x8c: return s->GPAFEN1;
	case 0x94: return s->GPPUD;
	case 0x98: return s->GPPUDCLK0;
	case 0x9c: return s->GPPUDCLK1;
	case 0x1c: /* GPSET0 (Write-Only)*/
	case 0x20: /* GPSET1 (Write-Only)*/
	case 0x28: /* GPCLR0 (Write-Only)*/
	case 0x2c: /* GPCLR1 (Write-Only)*/
        default:
            fprintf(stderr, "[QEMU][Raspi] Warning Read from unknown offset %x!\n", (unsigned int)offset);
            break;
    }
    return 0;
}



/* This fuction is called upon write detection */
/* Device states are updated according to the manipulated memory device */

static void bcm2835_todo_write(void *opaque, hwaddr offset,
    uint64_t value, unsigned size)
{
    bcm2835_todo_state *s = (bcm2835_todo_state *)opaque;
    switch (offset)
       {
	      case 0x00: s->GPFSEL0 = (value & 0xffffffff); break;
	      case 0x04: s->GPFSEL1 = (value & 0xffffffff); break;
	      case 0x08: s->GPFSEL2 = (value & 0xffffffff); break;
	      case 0x0c: s->GPFSEL3 = (value & 0xffffffff); break;
	      case 0x10: s->GPFSEL4 = (value & 0xffffffff); break;
	      case 0x14: s->GPFSEL5 = (value & 0xffffffff); break;
	      case 0x1c: s->GPSET0 = (value & 0xffffffff); break;
	      case 0x20: s->GPSET1 = (value & 0xffffffff); break;
	      case 0x28: s->GPCLR0 = (value & 0xffffffff); break;
	      case 0x2c: s->GPCLR1 = (value & 0xffffffff); break;
	      case 0x40: s->GPEDS0 = (value & 0xffffffff); break;
	      case 0x44: s->GPEDS1 = (value & 0xffffffff); break;
	      case 0x4c: s->GPREN0 = (value & 0xffffffff); break;
	      case 0x50: s->GPREN1 = (value & 0xffffffff); break;
	      case 0x58: s->GPFEN0 = (value & 0xffffffff); break;
	      case 0x5c: s->GPFEN1 = (value & 0xffffffff); break;
	      case 0x64: s->GPHEN0 = (value & 0xffffffff); break;
	      case 0x68: s->GPHEN1 = (value & 0xffffffff); break;
	      case 0x70: s->GPLEN0 = (value & 0xffffffff); break;
	      case 0x74: s->GPLEN1 = (value & 0xffffffff); break;
	      case 0x7c: s->GPAREN0 = (value & 0xffffffff); break;
	      case 0x80: s->GPAREN1 = (value & 0xffffffff); break;
	      case 0x88: s->GPAFEN0 = (value & 0xffffffff); break;
	      case 0x8c: s->GPAFEN1 = (value & 0xffffffff); break;
	      case 0x94: s->GPPUD = (value & 0xffffffff); break;
	      case 0x98: s->GPPUDCLK0 = (value & 0xffffffff); break;
	      case 0x9c: s->GPPUDCLK1 = (value & 0xffffffff); break;
	      case 0x34: /* GPLEV0 (Read-Only) */
	      case 0x38: /* GPLEV1 (Read-Only) */
	      default: goto err_out;
	}
    
    write_gpio(s); /* Set output levels and update simulator values */
    return;
	err_out:
    		fprintf(stderr, "[QEMU][Raspi] Warning Write to unknown offset %x!\n", (unsigned int)offset);
}

/*Tie the read/ write functions to bcm_todo*/
static const MemoryRegionOps bcm2835_todo_ops = {
    .read = bcm2835_todo_read,
    .write = bcm2835_todo_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const VMStateDescription vmstate_bcm2835_todo = {
    .name = TYPE_BCM2835_TODO,
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_END_OF_LIST()
    }
};

static int bcm2835_todo_init(SysBusDevice *sbd)
{
    //const int SERVER_PORT = 8989;

    DeviceState *dev = DEVICE(sbd);
    bcm2835_todo_state *s = BCM2835_TODO(dev);

    /* Modified by Yeqing Yan 
     * Change size to 0xAO */
    /* memory_region_init_io(&s->iomem, OBJECT(s), &bcm2835_todo_ops, s,
        TYPE_BCM2835_TODO, 0x1000000);*/
    memory_region_init_io(&s->iomem, OBJECT(s), &bcm2835_todo_ops, s,
        TYPE_BCM2835_TODO, 0xa0);
    sysbus_init_mmio(sbd, &s->iomem);

    vmstate_register(dev, -1, &vmstate_bcm2835_todo, s);

    s->socketfd = setup_socket();
    return 0;

}

/*Initialize the device memory and sysbus connection*/
                                                            
static void bcm2835_todo_class_init(ObjectClass *klass, void *data)
{
    SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);

    sdc->init = bcm2835_todo_init;
}

static TypeInfo bcm2835_todo_info = {
    .name          = TYPE_BCM2835_TODO,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(bcm2835_todo_state),
    .class_init    = bcm2835_todo_class_init,
};

static void bcm2835_todo_register_types(void)
{
    type_register_static(&bcm2835_todo_info);
}

type_init(bcm2835_todo_register_types)

