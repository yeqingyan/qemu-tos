/*
 * Raspberry Pi emulation (c) 2012 Gregory Estrade
 * This code is licensed under the GNU GPLv2 and later.
 */

#include "hw/sysbus.h"
#include <sys/types.h>         
#include <sys/socket.h>
//#include "qapi/qmp/qjson.h"
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
#define TERM_BIT_PIN 25
#define TERM_READ_PIN 7
#define TERM_WRITE_PIN 8
#define TOS_BIT_PIN 14
#define TOS_READ_PIN 18
#define TOS_WRITE_PIN 15

// GPIO Registers offset
#define GPFSEL0 0x0
#define GPFSEL1 0x4
#define GPFSEL2 0x8
#define GPFSEL3 0xC
#define GPFSEL4 0x10
#define GPFSEL5 0x14
#define GPSET0  0x1C
#define GPSET1  0x2D
#define GPCLR0  0x28
#define GPCLR1  0x2C
#define GPLEV0  0x34
#define GPLEV1  0x38


#define check_bit(var,pos)  ((var) & (1<<(pos)))

typedef struct {
    SysBusDevice busdev;
    MemoryRegion iomem;
    int socketfd;
} bcm2835_todo_state;


int current_pinDir =9; //random number other than 1 and 0

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

static uint64_t read_gpio(bcm2835_todo_state *s, unsigned int offset) {
    char buf[256];
    unsigned int value = 0;
    int n;

    if ((offset != GPLEV0) && (offset != GPLEV1)) {
        fprintf(stderr, "[QEMU][TOS] Warnning! GPIO read from unknown offset %x!", offset);
        return (uint64_t)0xffffffff;
    }
	
    /* Send msg to server */
    bzero(buf, 256);
	/***************** print level**************/
    sprintf(buf, "#R\n");

    n = write(s->socketfd, buf, strlen(buf));

    if (n < 0) {
        fprintf(stderr, "[QEMU][Raspi] ERROR %s writing to socket\n", strerror(errno));
        exit(1);
    }    

    // /* Read response */
    // bzero(buf, 256);
    // n = read(s->socketfd, buf, 255);

    // if (n < 0) {
    //     fprintf(stderr, "[QEMU][Raspi] READ ERROR %s reading from socket %s\n", strerror(errno), buf);
    //     exit(1);
    // }    

    // value = atoi(buf);
    fprintf(stderr, "\n[QEMU] TOS READ GPIO: read %d from %x\n", value, (unsigned int)offset);

    //fprintf(stderr, "[QEMU][TOS] Warnning! FIXME, add logic for read");
    return (uint64_t) value; 
}

static void write_gpio(bcm2835_todo_state *s, uint64_t value, short bit) {
    char buf[256];
    int n, pin; 
    
    pin = get_pin(value);
        /* Send msg to server */
    bzero(buf, 256);
    sprintf(buf, "PIN%dW%d\n", pin, bit);
	

    	/*Creating a json object*/
	json_object *jobj = json_object_new_object();

	/*Creating a json integer*/
	json_object *PinNo = json_object_new_int(pin);

	/*Creating a json boolean*/
	json_object *PinValue = json_object_new_boolean(bit);
	json_object *PinDir = json_object_new_int(current_pinDir);

	/*if(current_pinDir ==0)
		json_object *PinDir = "input";
	else if (current_pinDir ==1)
		json_object *PinDir = "output";
	else json_object *PinDir = "unknown pin function";*/

	/*Creating a json array*/
	json_object *jarray = json_object_new_array();

	
	/*Adding the above created json strings to the array*/
	json_object_array_add(jarray,PinNo);
	json_object_array_add(jarray,PinValue);
	json_object_array_add(jarray,PinDir);
	
	json_object_object_add(jobj,"Pin Details", jarray);
	printf("Size of JSON_TO_STRING- %lu,\n %s\n", sizeof(json_object_to_json_string(jobj)), json_object_to_json_string(jobj));
        n = write(s->socketfd, json_object_to_json_string(jobj), sizeof(json_object_to_json_string(jobj)));
    /* json code for filling array
      [ "PinNo" :
	"PinValue":
	"PinDir":    ]
	*/
	//PinDir will be always Output when you are writing to it

	

   // n = write(s->socketfd, buf, strlen(buf));

    if (n < 0) {
        fprintf(stderr, "[QEMU][Raspi] ERROR %s writing to socket\n", strerror(errno));
        exit(1);
    }    

    return;
}


/*
 * Read from socket 
 */
static uint64_t bcm2835_todo_read(void *opaque, hwaddr offset,
    unsigned size)
{
    uint64_t value;
    bcm2835_todo_state *s = (bcm2835_todo_state *)opaque;

    switch (offset) {
        case 0x0: //FS0
        case 0x4: //FS1
        case 0x8: //FS2
        case 0xC: //FS3
        case 0x10: //FS4
        case 0x14: //FS5
            return 0;
	case GPLEV0: 
	        value = read_gpio(s, (unsigned int)offset);break;
        case GPLEV1:
            value = read_gpio(s, (unsigned int)offset); break;
        case 0x94:  //GPPUD 
            value = 0;
        default:
            fprintf(stderr, "[QEMU][Raspi] Warning Read from unknown offset %x!\n", (unsigned int)offset);
            value = 0xffffffff;
    }
    return value;
}
int get_pin_from_GPFSEL( uint64_t value)
{	
	fprintf(stderr, "[QEMU][Raspi] in function value in hex = %x!\n", value);
    int pin=0;
    int n;
    for(n=0;n<30;n+3)
    {
	if(check_bit(value,n)) 
	{
		pin=pin+n;
		//break;
	}
    }	
    
    return pin;
}
/*
 * Write to socket 
 * Format: 
 *  Write to offest Cmd#Pin
 *  Response should be "OK"
 */
static void bcm2835_todo_write(void *opaque, hwaddr offset,
    uint64_t value, unsigned size)
{
    bcm2835_todo_state *s = (bcm2835_todo_state *)opaque;
    int pin=-1;
    switch (offset) {
        case GPFSEL0: //FS0  pin 0-9
		//current_pinDir = value;	
		//fprintf(stderr, "[QEMU][Raspi] GPSEL0 %d!\n", value);
		pin = get_pin_from_GPFSEL(value);
		fprintf(stderr, "[QEMU][Raspi] GPSEL0 pin=  %d!\n", pin);		
		break;

        case GPFSEL1:  //FS1 pin 10-19
		//current_pinDir = value;	
		//fprintf(stderr, "[QEMU][Raspi] GPSEL1 %d!\n", value);
		pin = get_pin_from_GPFSEL(value);
		pin = pin+10;
		fprintf(stderr, "[QEMU][Raspi] GPSEL1 pin=  %d!\n", pin);		
		break;

 
        case GPFSEL2: //FS2  pin 20-29
		//current_pinDir = value;	
		//fprintf(stderr, "[QEMU][Raspi] GPSEL2 %x!\n", value);
		pin = get_pin_from_GPFSEL(value);
		pin = pin+20;
		fprintf(stderr, "[QEMU][Raspi] GPSEL2 pin=  %d!\n", pin);		
		break;

        case GPFSEL3: //FS3  pin 30-39
        case GPFSEL4: //FS4  pin 40-49
        case GPFSEL5: //FS5  pin 50-53
            break ;

        case GPSET0:
            write_gpio(s, value, 1);  // Pin Output Set 0
	    //fprintf(stderr, "[QEMU][Raspi] Value at GPSET0 is %d!", value);
            break;
        case GPSET1:        
            fprintf(stderr, "[QEMU][Raspi] Warning! Write to Pin 32-53 not set!\n");
            break;
        case GPCLR0: 
	        // fprintf(stderr, "Pin Output Clear 0\n"); break;
            write_gpio(s, value, 0);  // Pin Output Clear 0
	    //fprintf(stderr, "[QEMU][Raspi] Value at GPCLR0 is %d!", value);
            break;
        case GPCLR1:
            fprintf(stderr, "[QEMU][Raspi] Warning! Write to Pin 32-53 not set!\n");
	    
            break;
        case 0x94:
            break;
        case 0x98:
            break;
        case 0x9C:
            break;
        default:
            fprintf(stderr, "[QEMU][Raspi] Warning Write to unknown offset %x!\n", (unsigned int)offset);
    }
    return;
}

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

