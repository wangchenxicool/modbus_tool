/*
 * _  _ ____ ___  ___  _  _ ____    ___ ____ ____ ___
 * |\/| |  | |  \ |__] |  | [__      |  |___ [__   |
 * |  | |__| |__/ |__] |__| ___]     |  |___ ___]  |
 *
 */
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <getopt.h>
#include <sys/timeb.h>
#include "modbus.h"

#define SLAVE         0x01

static int SPACE_TIME = 50;
static int STEP_MODE = 0;
static int COUNTS = 1;
static int WAIT_TIME = 0;

void print_usage (const char *prog)
{
    printf ("\nUsage: <%s serial_node  data1,data2,..., -scnh>\n\n", prog);
    puts ("  -s: modbus space time鋅n"
          "  -c: step run衆n"
          "  -n: repeat times\n"
          "  -w: wait time\n"
          "  -h: help\n");
    exit (1);
}

void parse_opts (int argc, char *argv[])
{
    int ret, ch;
    opterr = 0;
    char key_value[128];

    SPACE_TIME = 50;
    STEP_MODE = 0;
    COUNTS = 1;

    while ( (ch = getopt (argc, argv, "w:s:cn:h")) != EOF)
    {
        switch (ch)
        {
        case 's':
            SPACE_TIME = atoi (optarg);
            break;
        case 'w':
            WAIT_TIME = atoi (optarg);
            break;
        case 'c':
            STEP_MODE = 1;
            break;
        case 'n':
            COUNTS = atoi (optarg);
            break;
        case 'h':
        case '?':
        default:
            print_usage (argv[0]);
        }
    }
}

int main (int argc, char *argv[])
{
    int i, ret;
    uint8_t *tab_registers;
    modbus_param_t mb_param;

    struct timeb time_send; //记录发送时间
    struct timeb time_rcv; //记录接收时间
    float time_use; //记录modbus采集使用时间

    if (argc < 3)
    {
        print_usage (argv[0]);
    }

    /* RTU parity : none, even, odd */
    modbus_init_rtu (&mb_param, argv[1], 9600, "none", 8, 1, SLAVE);
    modbus_set_debug (&mb_param, TRUE);
    if (modbus_connect (&mb_param) == -1)
    {
        perror ("[modbus_connect]");
        exit (1);
    }

    /* Allocate and initialize the different memory spaces */
    tab_registers = (uint8_t *) malloc (512 * sizeof (uint8_t));
    memset (tab_registers, 0, 512 * sizeof (uint8_t));

    /* 填写modbus请求帧 */
    uint8_t query[128];
    int query_length = 0;
    char value[16];
    const char *lps = NULL, *lpe = NULL;
    lpe = lps = argv[2];
    i = 0;
    while (lps)
    {
        lps = strchr (lps, ',');
        if (lps)
        {
            memset (value, 0, sizeof (value));
            strncpy (value, lpe, lps - lpe);
            query[i++] = (unsigned char) strtol (value, NULL, 10);
            lps += 1;
            lpe = lps;
        }
    }
    query_length = i;

    parse_opts (argc, argv);

    for (i = 0; i < COUNTS; i++)
    {
        printf ("--------------------------------------\n");
        modbus_send (&mb_param, query, query_length);
        //serial_send (&mb_param, query, query_length);
        rcv_msg (&mb_param, tab_registers, 2000, WAIT_TIME);
        printf ("\n");
        wcx_sleep (0, SPACE_TIME * 1000);
        if (STEP_MODE == 1)
        {
            printf ("push enter key to continue...\n");
            getchar ();
        }
    }

    free (tab_registers);
    modbus_close (&mb_param);

    return 0;
}
