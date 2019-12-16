/*
 * main.c
 *
 *  Created on: Sep 25, 2019
 *      Author: zhanlei
 */
#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

#include <sys/ioctl.h>
#include <sys/epoll.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#define INT_IO_PIN			963

#define _T_TEXT(NUM)		#NUM
#define TO_TEXT(NUM)		_T_TEXT(NUM)

#define SIZEMIN				4 // no any argument
#define SIZEARG				32 // max argument 32 bytes
#define SIZELEN(x)          (x+SIZEMIN)
#define SIZEMAX             SIZELEN(SIZEARG)

// i2c string definition
#define CMD 0 // cmd offset
#define STA 1 // sta offset
#define LEN 2 // len offset
#define ARG 3 // arg offset

#define CHS(x)  (ARG+(x)) // chs offset

#define CMD_GET_FALLSNS		    0x0D // get fall sensor
#define CMD_GET_FALLSNS_DAT	    0x17
#define CMD_SET_FALLSNS_CLR	    0x18
#define CMD_GET_FALLSNS_FIFO    0x19
#define CMD_SET_FALLSNS_REG     0x20
#define CMD_GET_FALLSNS_REG     0x21

#define ACK_OK  0x00

#if 0
#define DEBUG_MSG(fmt, ...)     printf("MSG: "fmt, ##__VA_ARGS__)
#define DEBUG_PF(fmt, ...)     printf(fmt, ##__VA_ARGS__)
#else
#define DEBUG_MSG(fmt, ...)
#define DEBUG_PF(fmt, ...)
#endif

union un_sns_r_len {
    uint8_t val;
    uint8_t state;
};

struct st_sns_cmd {
    uint8_t id;
    uint8_t dln;
    uint8_t argc;
    uint8_t args[SIZEARG];
};

struct st_sns_cmd_ret {
    uint8_t state;
    uint8_t argc;
    uint8_t args[SIZEARG];
};

struct st_sns_point_data {
    int16_t x;
    int16_t y;
    int16_t z;
};

struct st_sns {
    uint16_t shock_counts;
    uint8_t fifo_counts;
    uint8_t data_counts;
};

struct st_sns_data {
    struct st_sns_point_data m_x;
    struct st_sns_point_data m_y;
    struct st_sns_point_data m_z;
};

int i2c_fd = -1;
int int_io_fd = -1;

uint8_t buf[SIZEMAX];

uint8_t checksum(const uint8_t *buf) {
    int i = buf[LEN] > SIZEARG ? SIZEMAX : SIZELEN(buf[LEN]), ret = 0;
    for (i--; i-- > 0;)
        ret += buf[i];
    return ret;
}

int i2c_init(void) {
    int ret = -1;
    i2c_fd = open("/dev/i2c-0", O_RDWR);
    if (i2c_fd > 0) {
        if (ioctl(i2c_fd, I2C_SLAVE, 0x48) == 0) {
            ret = 0;
        } else {
            printf("set i2c slave device address failed\n");
            close(i2c_fd);
            i2c_fd = -1;
        }
    } else {
        printf("open i2c device failed\n");
    }

    return ret;
}

int gpio_init(void) {
    int ret = -1;
    if (access("/sys/class/gpio/gpio"TO_TEXT(INT_IO_PIN), F_OK) == -1) {
        int fd = open("/sys/class/gpio/export", O_WRONLY);
        if (fd > 0) {
            const int len = strlen(TO_TEXT(INT_IO_PIN));
            int nbyte = write(fd, TO_TEXT(INT_IO_PIN), len);
            if (nbyte == len) {
                ret = 0;
            } else {
                printf("write "TO_TEXT(INT_IO_PIN)" to export failed\n");
            }
            close(fd);
        } else {
            printf("open export file failed\n");
        }
    } else {
        ret = 0;
    }

    if (ret == 0) {
        ret = -1;
        int fd = open("/sys/class/gpio/gpio"TO_TEXT(INT_IO_PIN)"/edge",
            O_WRONLY);
        if (fd > 0) {
            int nbyte = write(fd, "falling", 8);
            if (nbyte == 8) {
                ret = 0;
            } else {
                printf("set IO"TO_TEXT(INT_IO_PIN)" edge to falling failed\n");
            }
            close(fd);
        }
    }

    if (ret == 0) {
        ret = -1;
        int fd = open("/sys/class/gpio/gpio"TO_TEXT(INT_IO_PIN)"/direction",
            O_WRONLY);
        if (fd > 0) {
            int nbyte = write(fd, "in", 3);
            if (nbyte == 3) {
                ret = 0;
            } else {
                printf("set IO"TO_TEXT(INT_IO_PIN)" direction to in failed\n");
            }
            close(fd);
        }
    }

    if (ret == 0) {
        int_io_fd = open("/sys/class/gpio/gpio"TO_TEXT(INT_IO_PIN)"/value",
            O_RDONLY);
        if (int_io_fd < 0) {
            ret = -1;
            printf("open IO"TO_TEXT(INT_IO_PIN)" value file failed\n");
        }
    }
    return ret;
}

void print_buf(const uint8_t *_buf, int size) {
    for (int i = 0; i < size; i++) {
        DEBUG_PF("%02X", _buf[i]);
        if (i + 1 >= size) {
            DEBUG_PF("\n");
        } else {
            DEBUG_PF(" ");
        }
    }
}

int parse_cmd_ret(const struct st_sns_cmd *cmd, const uint8_t *dat,
    struct st_sns_cmd_ret *results) {
    int ret = -1;
    results->argc = dat[LEN];
    if (checksum(dat) == dat[CHS(results->argc)]) {
        results->state = dat[STA];
        if (ACK_OK == results->state) {
            memcpy(results->args, &dat[ARG], results->argc);
            ret = 0;
        }
    }
    return ret;
}

int run_cmd(const struct st_sns_cmd *cmd, struct st_sns_cmd_ret *results) {
    int ret = -1;
    int wbyte;

    // encode
    buf[CMD] = cmd->id;
    buf[STA] = cmd->dln;
    buf[LEN] = cmd->argc;
    if (cmd->argc > 0) {
        memcpy(&buf[ARG], cmd->args, cmd->argc);
    }
    buf[CHS(cmd->argc)] = checksum(buf);
    // send
    wbyte = write(i2c_fd, &buf, SIZELEN(buf[LEN]));
    DEBUG_MSG("send: ");
    print_buf(buf, SIZELEN(buf[LEN]));
    if (wbyte == SIZELEN(buf[LEN])) {
        // MCU prepares data
        usleep(50000);
        int rbyte = read(i2c_fd, buf, SIZELEN(cmd->dln));
        DEBUG_MSG("write ok\n");
        if (rbyte > 0) {
            DEBUG_MSG("read ok\n");
            DEBUG_MSG("recv: ");
            print_buf(buf, rbyte);
            ret = parse_cmd_ret(cmd, buf, results);
        } else {
            DEBUG_MSG("rbyte: %d\n", rbyte);
        }
    } else {
        DEBUG_MSG("wbyte: %d\n", wbyte);
    }
    return ret;
}

int cmd_get_fall_sns(struct st_sns *sns) {
    const struct st_sns_cmd cmd = {
        CMD_GET_FALLSNS, 4, 0, { 0 } };
    struct st_sns_cmd_ret results;
    int ret = run_cmd(&cmd, &results);
    if (ret == 0) {
        if (results.argc > 3) {
            sns->data_counts = results.args[0];
            sns->fifo_counts = results.args[1];
            sns->shock_counts = results.args[2] | (((uint16_t) results.args[3]) << 8);
        } else {
            ret = -1;
        }
    }
    return ret;
}

int cmd_get_fall_sns_dat(uint8_t index, struct st_sns_data *data) {
    const struct st_sns_cmd cmd = {
        CMD_GET_FALLSNS_DAT, 18, 1, { index } };
    struct st_sns_cmd_ret results;
    int ret = run_cmd(&cmd, &results);
    if (ret == 0) {
        if (results.argc > 17) {
            uint8_t i = 0;
            data->m_x.x = (int16_t) (results.args[i] | (((uint16_t) results.args[i + 1]) << 8));
            i += 2;
            data->m_x.y = (int16_t) (results.args[i] | (((uint16_t) results.args[i + 1]) << 8));
            i += 2;
            data->m_x.z = (int16_t) (results.args[i] | (((uint16_t) results.args[i + 1]) << 8));
            i += 2;

            data->m_y.x = (int16_t) (results.args[i] | (((uint16_t) results.args[i + 1]) << 8));
            i += 2;
            data->m_y.y = (int16_t) (results.args[i] | (((uint16_t) results.args[i + 1]) << 8));
            i += 2;
            data->m_y.z = (int16_t) (results.args[i] | (((uint16_t) results.args[i + 1]) << 8));
            i += 2;

            data->m_z.x = (int16_t) (results.args[i] | (((uint16_t) results.args[i + 1]) << 8));
            i += 2;
            data->m_z.y = (int16_t) (results.args[i] | (((uint16_t) results.args[i + 1]) << 8));
            i += 2;
            data->m_z.z = (int16_t) (results.args[i] | (((uint16_t) results.args[i + 1]) << 8));
        } else {
            ret = -1;
        }
    }
    return ret;
}

int cmd_set_fall_sns_clr(void) {
    const struct st_sns_cmd cmd = {
        CMD_SET_FALLSNS_CLR, 0, 0, { 0 } };
    struct st_sns_cmd_ret results;
    return run_cmd(&cmd, &results);
}

int cmd_get_fall_sns_fifo(uint8_t index, struct st_sns_point_data *p_data) {
    const struct st_sns_cmd cmd = {
        CMD_GET_FALLSNS_FIFO, 6, 1, { index } };
    struct st_sns_cmd_ret results;
    int ret = run_cmd(&cmd, &results);
    if (ret == 0) {
        if (results.argc > 5) {
            uint8_t i = 0;
            p_data->x = (int16_t) (results.args[i] | (((uint16_t) results.args[i + 1]) << 8));
            i += 2;
            p_data->y = (int16_t) (results.args[i] | (((uint16_t) results.args[i + 1]) << 8));
            i += 2;
            p_data->z = (int16_t) (results.args[i] | (((uint16_t) results.args[i + 1]) << 8));
        } else {
            ret = -1;
        }
    }
    return ret;
}

int cmd_set_fall_sns_reg(uint8_t addr, uint8_t val) {
    const struct st_sns_cmd cmd = {
        CMD_SET_FALLSNS_REG, 0, 2, { addr, val } };
    struct st_sns_cmd_ret results;
    return run_cmd(&cmd, &results);
}

int cmd_get_fall_sns_reg(uint8_t addr, uint8_t *val) {
    const struct st_sns_cmd cmd = {
        CMD_GET_FALLSNS_REG, 1, 1, { addr } };
    struct st_sns_cmd_ret results;
    int ret = run_cmd(&cmd, &results);
    if (ret == 0) {
        *val = results.args[0];
    }
    return ret;
}

void process_event() {
    struct st_sns_data data[32];
    struct st_sns_point_data p_data[32];
    struct st_sns sns = {0, 0, 0};
    static uint8_t flag = 0;
    if (cmd_get_fall_sns(&sns) == 0) {
        uint8_t max_counts = sns.data_counts >= 32? 32 : sns.data_counts;
        uint8_t id = 0, reg_val = 0;
        for (uint16_t i = 0; i < max_counts; i++) {
            if (i >= 32) {
                break;
            }
            if (cmd_get_fall_sns_dat(i, &data[i]) == 0) {

            } else {
                max_counts = i;
                break;
            }
        }

        if (cmd_get_fall_sns_reg(0x00, &id) == 0) {
            printf("sns id       : 0x%02X\n", id);
        }

        if (cmd_get_fall_sns_reg(0x21, &reg_val) == 0) {
            printf("reg start    : 0x%02X\n", reg_val);
            if (cmd_set_fall_sns_reg(0x21, 0x55) == 0) {
                uint8_t tmp = 0;
                if (cmd_get_fall_sns_reg(0x21, &tmp) == 0) {
                    printf("reg changed  : 0x%02x\n", tmp);
                }
                cmd_set_fall_sns_reg(0x21, reg_val);
            }
        }

        printf("data counts  : %u\n", sns.data_counts);
        printf("fifo counts  : %u\n", sns.fifo_counts);
        printf("shock counts : %u\n", sns.shock_counts);
        printf("============================\n");
        printf("NO | [mx]  [my]  [mz]\n");
        printf("---+ ----  ----  ----\n");
        for (uint16_t i = 0; i < max_counts; i++) {
            printf("%02u | [%-4d  %-4d  %-4d]", i, data[i].m_x.x, data[i].m_x.y, data[i].m_x.z);
            printf(" [%-4d  %-4d  %-4d]", data[i].m_y.x, data[i].m_y.y, data[i].m_y.z);
            printf(" [%-4d  %-4d  %-4d]\n", data[i].m_z.x, data[i].m_z.y, data[i].m_z.z);
        }
        max_counts = sns.fifo_counts >= 32? 32 : sns.fifo_counts;
        for (uint16_t i = 0; i < max_counts; i++) {
            if (i >= 32) {
                break;
            }
            if (cmd_get_fall_sns_fifo(i, &p_data[i]) == 0) {

            } else {
                max_counts = i;
                break;
            }
        }
        printf("------------------------\n");
        printf("NO |   x      y      z\n");
        printf("---+ -----  -----  -----\n");
        for (uint16_t i = 0; i < max_counts; i++) {
            printf("%02u | %-5d  %-5d  %-5d\n", i, p_data[i].x, p_data[i].y, p_data[i].z);
        }
        if (sns.data_counts >= 32) {
            if (flag) {
                cmd_set_fall_sns_clr();
                printf("MSG : clear sns shock history\n");
                flag = 0;
            } else {
                flag = 1;
                printf("MSG : Keep history data\n");
            }
        }
    }
}

int main(int argc, const char *argv[]) {
    int epfd = epoll_create1(0);
    if (epfd == -1) {
        printf("create epoll failed\n");
        goto exit_check;
    }

    if ((i2c_init() == 0) && (gpio_init() == 0)) {
        struct epoll_event ev;
        struct epoll_event events;
        ev.events = EPOLLPRI | EPOLLERR;
        ev.data.fd = int_io_fd;

        if (epoll_ctl(epfd, EPOLL_CTL_ADD, int_io_fd, &ev) == 0) {
            char val[2];
            printf("start epoll loop\n");
            while (1) {
                int n = epoll_wait(epfd, &events, 1, 50000);
                if (n > 0) {
                    // can read GPIO value
                    lseek(int_io_fd, 0, SEEK_SET);
                    val[0] = '-';
                    read(int_io_fd, val, 2);
                    printf("GPIO: %c\n", val[0]);
                    process_event();
                    printf("------------ End Of GPIO ISR ------------\n");
                } else if (n == -1) {
                    printf("epoll wait failed\n");
                    break;
                } else {
                    process_event();
                    printf("============= End Of Timeout =============\n");
                }
            }
        } else {
            printf("add interrupt GPIO to epoll failed\n");
            goto exit_check;
        }
    }

    exit_check: if (i2c_fd > 0) {
        close(i2c_fd);
    }

    if (int_io_fd) {
        close(int_io_fd);
    }

    if (epfd > 0) {
        close(epfd);
    }
    return 0;
}
