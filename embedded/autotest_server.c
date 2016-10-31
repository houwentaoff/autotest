/*
 * server.c
 *
 *  Created on: Oct 9, 2012
 *      Author: hmng
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/wait.h>
#include <signal.h>
#include "jsonrpc-c.h"

#define PORT 1234  // the port users will be connecting to

struct jrpc_server my_server;
typedef struct 
{
    int key_code;
    const char*name;
}key_map_t;

static key_map_t key_map[] = {
    /* 左边字符键盘 */
    {0x05, "37"},  {0x04, "38"}, {0x03, "39"}, {0x02, "40"}, {0x01, "41"}, {0x00, "42"},
    {0xd, "31"},  {0xc, "32"}, {0xb, "33"}, {0xa, "34"}, {0x9, "35"}, {0x8, "36"},
    {0x15, "25"},  {0x14, "26"}, {0x13, "27"}, {0x12, "28"}, {0x11, "29"}, {0x10, "30"},
    {0x1d, "19"},  {0x1c, "20"}, {0x1b, "21"}, {0x1a, "22"}, {0x19, "23"}, {0x18, "24"},
    {0x35, "13"},  {0x34, "14"}, {0x33, "15"}, {0x32, "16"}, {0x31, "17"}, {0x30, "18"},
    {0x2d, "7"},  {0x2c, "8"}, {0x2b, "9"}, {0x2a, "10"}, {0x29, "11"}, {0x28, "12"},
    {0x25, "1"},  {0x24, "1"}, {0x23, "3"}, {0x22, "4"}, {0x21, "5"}, {0x20, "6"},
    /* 右边功能键盘 */
    {0xbe, "auto"},  {0xb6, "lock"}, {0xae, "pc"}, {0xa6, "quit"},
    {0xbd, "peel"},  {0xb5, "f1"}, {0xad, "plu"}, {0xa5, "clear"},
    {0xbc, "f2"},  {0xb4, "f3"}, {0xac, "f4"}, {0xa4, "func"},
    {0xbb, "num7"},  {0xb3, "num8"}, {0xab, "num9"}, {0xa3, "zero"},
    {0xba, "num4"},  {0xb2, "num5"}, {0xaa, "num6"}, {0xa2, "count"},
    {0xb9, "num1"},  {0xb1, "num2"}, {0xa9, "num3"}, {0xa1, "ok"},
    {0xb8, "num0"},  {0xb0, "dot"}, {0xa8, "paper"}, {0xa0, "print"},
    NULL,
};
#if 0
        [ '37', '38', '39', '40', '41', '42',     'auto', 'lock', 'pc' , 'quit' ,
            '31', '32', '33', '34', '35', '36',     'peel', 'f1'  , 'plu', 'clear',  
            '25', '26', '27', '28', '29', '30',     'f2'  , 'f3'  , 'f4' , 'func' ,
            '19', '20', '21', '22', '23', '24',     'num7', 'num8', 'num9', 'zero',
            '13', '14', '15', '16', '17', '18',     'num4', 'num5', 'num6', 'count',
            '7',  '8' , '9' , '10', '11', '12',     'num1', 'num2', 'num3', 'ok',
            '1',  '2' , '3' ,  '4', '5' , '6' ,     'num0', 'dot' , 'paper', 'print',
        ];
#endif
static cJSON * say_hello(jrpc_context * ctx, cJSON * params, cJSON *id) {
	return cJSON_CreateString("Hello!");
}

static cJSON * exit_server(jrpc_context * ctx, cJSON * params, cJSON *id) {
	jrpc_server_stop(&my_server);
	return cJSON_CreateString("Bye!");
}

static cJSON * press_key(jrpc_context * ctx, cJSON * params, cJSON *id);
/*
20150101
just a simple input test code
lei_wang
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <linux/input.h>
/**
 * @brief 
 *
 * @param fd
 */
void event_sync(int fd, int key_code)
{
    struct input_event event;
    static int i=0;

    if (fd <= fileno(stderr)) 
    {
        return;
    }
    memset(&event, 0, sizeof(event));
    gettimeofday(&event.time, NULL);

    event.type = EV_SYN;
    /* EV_SYN + SYN_CONFIG 会触发input_dev->event()函数 input_dev->grab->handler->event()还不知怎么触发
     * EV_SYN + SYN_REPORT 不会触发input_dev->event() 
     * */
    event.code = SYN_CONFIG;// 引起快速中断？...SYN_REPORT; 但是read会延迟
    event.value = key_code;//0;//用这个 传value 不用EV_KEY中的code和value
    if (write(fd, &event, sizeof(event)) < 0) 
    {
        perror("send_event error\n");
    }
}
void send_event(int fd, unsigned short type, unsigned short code, int value) 
{
    struct input_event event;
    int len;

    printf("SendEvent call (%d,%d,%d,%d)\n", fd, type, code, value);

    if (fd <= fileno(stderr)) return;

    memset(&event, 0, sizeof(event));
    gettimeofday(&event.time, NULL);

    // event (type, code, value)
    event.type = type;
    event.code = code;
    event.value = value;
    if (write(fd, &event, sizeof(event)) < 0) {
        perror("send_event error\n");
    }
#if 0
    // sync (0,0,0)
    event.type = EV_SYN;
    event.code = SYN_CONFIG;//SYN_REPORT;
    event.value = 0;
    if (write(fd, &event, sizeof(event)) < 0) {
        perror("send_event error\n");
    }
#endif
}
static int get_keycode(const char* name)
{
    int key_code = 0x0;
    key_map_t *pmap_node = key_map;
    
    if (!name)
    {
        return (-1);
    }
    for (; pmap_node!=NULL; pmap_node++)
    {
        if (strcmp(pmap_node->name, name) == 0)
        {
            key_code = pmap_node->key_code;
            break;
        }
    }
    if (!pmap_node)
    {
        return -1;
    }
    return key_code;
}

static cJSON * press_key(jrpc_context * ctx, cJSON * params, cJSON *id) 
{
	int fd;
	int version;
	int ret;
	struct input_event ev;
	int key_code = 0;
    const char *name = NULL;

    if (!params || !params->child)
    {
        return cJSON_CreateString("no param!");
    }
    if (cJSON_String == params->child->type)
    {
        name = params->child->valuestring;
    }
    else
    {
        return cJSON_CreateString("param error!");
    }
    key_code = get_keycode(name);
	fd = open("/dev/input/event0", O_RDWR);
	if (fd < 0) {
		printf("open file failed\n");
		exit(1);
	}

	ioctl(fd, EVIOCGVERSION, &version);
	printf("evdev driver version is 0x%x: %d.%d.%d\n",
					version, version>>16, (version>>8) & 0xff, version & 0xff);
    event_sync(fd, key_code);
	close(fd);
	return cJSON_CreateString("success!");
}

int main(void) {
	jrpc_server_init(&my_server, PORT);
	jrpc_register_procedure(&my_server, say_hello, "sayHello", NULL );
	jrpc_register_procedure(&my_server, exit_server, "exit", NULL );
	jrpc_register_procedure(&my_server, press_key, "pressKey", NULL );
	jrpc_server_run(&my_server);
	jrpc_server_destroy(&my_server);
	return 0;
}
