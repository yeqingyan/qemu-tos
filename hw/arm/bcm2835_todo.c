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

#define SERVER_PORT 8988
#define GPIO_MEM_SIZE 0xA0
#define GPIO_REG_SIZE  32
#define GPIO_TOTAL_PIN 54
#define INPUT 0x00
#define OUTPUT 0x01
#define SETBIT(reg,bit) (reg |= (1ULL << (bit)))
#define CLRBIT(reg,bit) (reg &= (~(1ULL << (bit))))

typedef struct {
    SysBusDevice busdev;
    MemoryRegion iomem;
    int socketfd;

    uint32_t GPFSEL[6];
   /* GPFSEL[0] : 0x00 Function Select Pins 00-09   
      GPFSEL[1] : 0x04 Function Select Pins 10-19 
      GPFSEL[2] : 0x08 Function Select Pins 20-29 
      GPFSEL[3] : 0x0c Function Select Pins 30-39 
      GPFSEL[4] : 0x10 Function Select Pins 40-49 
      GPFSEL[5] : 0x14 Function Select Pins 50-53 */

    uint32_t GPSET[2];   
  /* GPSET[0] : 0x1c Output Set Pins  0-31 [0=No Effect | 1=Set] 
	 GPSET[1] : 0x20 Output Set Pins 32-53 [0=No Effect | 1=Set] */

    uint32_t GPCLR[2];   
  /* GPCLR[0] : 0x28 Output Clear Pins   0-31 [0=No Effect | 1=Clear]    
	 GPCLR[1] : 0x2c Output Clear Pins  32-53 [0=No Effect | 1=Clear] */

    uint32_t GPLEV[2];    
 /* GPLEV[0] : 0x34 Pin Level (Read Only)  0-31 [0=Low | 1=High] 
    GPLEV[1] : 0x38 Pin Level (Read Only) 32-53 [0=Low | 1=High]  */

    uint32_t GPEDS0;    // 0x40 Event Detect Pins  0-31 [0=Event Not Detected | 1=Event Detected] 
    uint32_t GPEDS1;    // 0x44 Event Detect Pins 32-53 [0=Event Not Detected | 1=Event Detected] 

    uint32_t GPREN0;    // 0x4c Rising Edge Detect Enable Pins  0-31 [0=Rising Edge Detect Disabled | 1=Rising Edge Detect Enabled] 
    uint32_t GPREN1;    // 0x50 Rising Edge Detect Enable Pins 32-53 [0=Rising Edge Detect Disabled | 1=Rising Edge Detect Enabled] 

    uint32_t GPFEN0;    // 0x58 Falling Edge Detect Enable Pins  0-31 [0=Falling Edge Detect Disabled | 1=Falling Edge Detect Enabled] 
    uint32_t GPFEN1;    // 0x5c Falling Edge Detect Enable Pins 32-53 [0=Falling Edge Detect Disabled | 1=Falling Edge Detect Enabled] 

    uint32_t GPHEN0;    // 0x64 High Level Detect Enable Bits  0-31 [0=High Level Detect Disabled | 1=High Level Detect Enabled] 
    uint32_t GPHEN1;    // 0x68 High Level Detect Enable Bits 32-53 [0=High Level Detect Disabled | 1=High Level Detect Enabled] 
   
    uint32_t GPLEN0;    // 0x70 Low Level Detect Enable Bits  0-31 [0=Low Level Detect Disabled | 1=Low Level Detect Enabled] 
    uint32_t GPLEN1;    // 0x74 Low Level Detect Enable Bits 32-53 [0=Low Level Detect Disabled | 1=Low Level Detect Enabled] 
    
    uint32_t GPAREN0;   // 0x7c Async Rising Edge Detect Enable Pins  0-31 [1=Async Rising Edge Detect Disabled | 1=Async Rising Edge Detect Enabled] 
    uint32_t GPAREN1;   // 0x80 Async Rising Edge Detect Enable Pins 32-53 [1=Async Rising Edge Detect Disabled | 1=Async Rising Edge Detect Enabled] 
    
    uint32_t GPAFEN0;   // 0x88 Async Falling Edge Detect Enable Pins  0-31 [1=Async Falling Edge Detect Disabled | 1=Async Falling Edge Detect Enabled] 
    uint32_t GPAFEN1;   // 0x8c Async Falling Edge Detect Enable Pins 32-53 [1=Async Falling Edge Detect Disabled | 1=Async Falling Edge Detect Enabled] 
    
    uint32_t GPPUD;     // 0x94 Pull-Up/Pull-Down All Pins [Bits 1-0: 0=Disable Pull-Up/Pull-Down | 1=Enable Pull-Down Control | 2=Enable Pull-Up Control] 
    
    uint32_t GPPUDCLK[2]; 
  /* GPPUDCLK[0] : 0x98 Pull-Up/Pull-Down Clock Pins  0-31 [0=No Effect | 1=Assert Clock on Line] 
     GPPUDCLK[1] : 0x9c Pull-Up/Pull-Down Clock Pins 32-53 [0=No Effect | 1=Assert Clock on Line]  */

    uint64_t PINSET; // Derived output state for pins 0-53 based on SET and CLR registers 
	uint64_t PINDIR; // Derived direction state for pins 0-53 based on GPFSEL registers 
} bcm2835_todo_state;

static int setup_socket(void) {
    struct sockaddr_in serv_addr;
    struct hostent *local;
    int socketfd;
    fprintf(stderr, "Connecting to Server..\n");
    local = gethostbyname("localhost");
    if (local == NULL) {
        fprintf(stderr, "[QEMU][Raspi] Cannot get hostname");
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

/**
 * Return pin function
 * @param s     Device State
 * @param pin   Pin Number
 * @return      Current pin function selection
 */

static uint32_t get_function (bcm2835_todo_state *s, int pin){
    int index = pin / 10;
    return ((s->GPFSEL[index] >> (3*(pin-(index*10))) & 0x7));
}

static void print_registers(bcm2835_todo_state *s)
{
	for (int i=0; i<6; i++){
		fprintf(stderr, "\n[QEMU][Raspi] s->GPFSEL[%d]  = %d!\n", i, s->GPFSEL[i]);
	}
	fprintf(stderr, "[QEMU][Raspi] s->PINSET  = %d!\n", s->PINSET);

}
static void send_json_to_simulator(bcm2835_todo_state *s, bool Read_Status)
{
	char PINSET[30],PINDIR[30];
	json_object *jobj = json_object_new_object(); //Creating a json object to send to GPIO Simulator
	sprintf(PINDIR,"%d", s->PINDIR); 
	sprintf(PINSET,"%d", s->PINSET);
	json_object_object_add(jobj,"PINDIR", json_object_new_string(PINDIR));
	json_object_object_add(jobj,"PINSET", json_object_new_string(PINSET));
	json_object_object_add(jobj,"READ", json_object_new_boolean(Read_Status)); // this field is used by Simulator to decide whether to send back PINSET values or not

	const char * string = json_object_to_json_string(jobj);
	int n = write(s->socketfd, string, strlen(string)); // Send the JSON String to GPIO Simulator
   	n = write(s->socketfd, "\n", 1);
	if (n < 0)  {
		fprintf(stderr, "[QEMU][Raspi] ERROR %s Writing to socket\n", strerror(errno));
    }    
}

static uint64_t read_gpio(bcm2835_todo_state *s) {
    char buf[256];
    uint64_t GPLEV_64=0,mask;
    bzero(buf, 256);  //places 256 null bytes in the string buf
    
	// Send a json string to GPIO Simulator
	send_json_to_simulator(s, true); // Parameter is true since we expect PINSET value in return from Simulator

	/*Read from Socket the PINSET values*/
   	while(read(s->socketfd, buf, 255) <= 0);
	GPLEV_64 = strtoull(buf, NULL, 10); // Converting received string to decimal value
	
	for (int i=0; i<GPIO_TOTAL_PIN; i++){
	    mask = 1<<i;
		if (get_function(s,i) == INPUT){ // only check input pins
			if ((mask & GPLEV_64) > 0) 	
				s->GPLEV[i/GPIO_REG_SIZE] |= mask;
			else 
				s->GPLEV[i/GPIO_REG_SIZE] &= ~mask;
		}
	}
}
 
static void write_gpio(bcm2835_todo_state *s){
    uint32_t mask;
	// adding pin values to PINSET and PINDIR
	for (int i=0; i < GPIO_TOTAL_PIN; i++)
   	{
		mask = (i/GPIO_REG_SIZE==0) ? (1<<i) : (i<<(i-GPIO_REG_SIZE));
		if (get_function(s,i) == OUTPUT) // making sure its an output 
		{   
			SETBIT(s->PINDIR,i);
			if (s->GPSET[i/GPIO_REG_SIZE] & mask) // set flag is set 
			{          
		        s->PINSET |= mask;            // set the state bit 
		        s->GPSET[i/GPIO_REG_SIZE] &= ~mask;           // clear the set bit 
			}
			else if (s->GPCLR[i/GPIO_REG_SIZE] & mask)  // clear flag is set
			{      
		        s->PINSET &= ~mask;            // set the state bit 
		        s->GPCLR[i/GPIO_REG_SIZE] &= ~mask;           // clear the set bit 
			}
        }
		else if (get_function(s,i) == INPUT)
			CLRBIT(s->PINDIR,i);
	}
	//print_registers(s);
	// Send a json string to GPIO Simulator
	send_json_to_simulator(s,false);	// Parameter is false since we do not expect any values in return
    return;
}


// This fuction is called upon read detection 
// Returns the device state field corresponding to the read address 

static uint64_t bcm2835_todo_read(void *opaque, hwaddr offset,unsigned size)
{
    uint64_t value;
    bcm2835_todo_state *s = (bcm2835_todo_state *)opaque;
   
   switch (offset) {
	case 0x00: 
	case 0x04: 
	case 0x08: 
	case 0x0c: 
	case 0x10:
	case 0x14: return s->GPFSEL[offset/4];
	
	case 0x34: 
	case 0x38: read_gpio(s);//read from gpio simulator
			   return s->GPLEV[(offset/4)-13];

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
	case 0x98: 
	case 0x9c: return s->GPPUDCLK[(offset/4)-38];
	case 0x1c: // GPSET0 (Write-Only)
	case 0x20: // GPSET1 (Write-Only)
	case 0x28: // GPCLR0 (Write-Only)
	case 0x2c: // GPCLR1 (Write-Only)
        default:
            fprintf(stderr, "[QEMU][Raspi] Warning Read from unknown offset %x!\n", (unsigned int)offset);
            break;
    }
    return 0;
}


// This fuction is called upon write detection 
// Device states are updated according to the manipulated memory device 

static void bcm2835_todo_write(void *opaque, hwaddr offset,
    uint64_t value, unsigned size)
{
    bcm2835_todo_state *s = (bcm2835_todo_state *)opaque;
    switch (offset)
       {
		case 0x00:
		case 0x04:
		case 0x08:
		case 0x0c:
		case 0x10:
		case 0x14: s->GPFSEL[offset/4] = (value & 0xffffffff);break;
		case 0x1c: 
		case 0x20: s->GPSET[(offset/4)-7] = (value & 0xffffffff);break;
		case 0x28:
		case 0x2c: s->GPCLR[(offset/4)-10] = (value & 0xffffffff); break;
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
		case 0x98: 
		case 0x9c: s->GPPUDCLK[(offset/4)-38] = (value & 0xffffffff); break;
		case 0x34: // GPLEV0 (Read-Only) 
		case 0x38: // GPLEV1 (Read-Only) 
		default: goto error;
	}
    
    write_gpio(s); // Set output levels and update simulator values 
    return;
	error:
    	fprintf(stderr, "[QEMU][Raspi] Warning write to unknown offset %x!\n", (unsigned int)offset);
}

//Tie the read/ write functions to bcm_todo
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
    DeviceState *dev = DEVICE(sbd);
    bcm2835_todo_state *s = BCM2835_TODO(dev);

    // Modified by Yeqing Yan 
    // Change size to 0xAO 
    // memory_region_init_io(&s->iomem, OBJECT(s), &bcm2835_todo_ops, s,TYPE_BCM2835_TODO, 0x1000000);
    memory_region_init_io(&s->iomem, OBJECT(s), &bcm2835_todo_ops, s,
        TYPE_BCM2835_TODO, 0xa0);
    sysbus_init_mmio(sbd, &s->iomem);
    vmstate_register(dev, -1, &vmstate_bcm2835_todo, s);
    s->socketfd = setup_socket();
    return 0;

}

//Initialize the device memory and sysbus connection                                           
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


