#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <string.h>
#include <termios.h>

#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <string>
#include <map>
#include <memory>

#define SEND_FILE_NAME  "./xindi_480_272.tft"
#define INIT_BAUD		115200
#define TRANSFER_BAUD 	230400

std::ifstream tftfile;
int tft_buff = 4096;
int tft_start;
int tft_end;
std::string tft_s;
std::string tft_data;
int tft_len;
int filesize;

bool send_finish = true;

int tft_index;
char tft_buffer[4096];

int fd;         // 串口描述符

void parse_cmd(char *cmd);
int set_option(int fd, int baudrate, int bits, unsigned char parity, unsigned char stopbit);

void init_download_to_screen();
void download_to_screen();
void send_cmd_download_data(int fd, std::string data);
void send_cmd_download(int fd, int filesize);

int main(int argc, char** argv) {
    

    int count;

    char buff[4096];

    if ((fd = open("/dev/ttyS0", O_RDWR | O_NDELAY | O_NOCTTY)) < 0) {
        printf("OPEN TTY failed\n");
        return 0;
    } else {
        set_option(fd, INIT_BAUD, 8, 'N', 1);
        fcntl(fd, F_SETFL, FNDELAY);
        if (access(SEND_FILE_NAME, F_OK) == 0) {
            init_download_to_screen();
            send_cmd_download(fd, filesize);
			if(TRANSFER_BAUD != INIT_BAUD)
				set_option(fd, TRANSFER_BAUD, 8, 'N', 1);
			printf("open file ok\n");
        }
		else
			printf("open file fail\n");
    }

    while (!send_finish)
    {
		
        if ((count = read(fd, buff, sizeof(buff))) > 0) {
            char *cmd = buff;
            parse_cmd(cmd);
            memset(buff, 0, sizeof(buff));
        }

        usleep(5000);
    }
    close(fd);
    return 0;
}

void parse_cmd(char *cmd) {
    std::cout << "Receive " << (int)cmd[0] << std::endl;
    switch (cmd[0])
    {
    case 0x5:
        download_to_screen();
        break;
    
    default:
        break;
    }
}

int set_option(int fd, int baudrate, int bits, unsigned char parity, unsigned char stopbit) {

    // time_t start_time = 0, end_time = 0;

    // time(&start_time);
    struct termios newtio, oldtio;

    if (tcgetattr(fd, &oldtio) != 0) {
        // printf("读取串口参数发生错误\n");
        return -1;
    }

    memset(&newtio, 0, sizeof(struct termios));
    cfmakeraw(&newtio);                         // 设置为原始模式

    /* 使能接收 */
    newtio.c_cflag |= CREAD;

    speed_t speed;

    // 数据位相关的比特位清零
    newtio.c_cflag &= ~CSIZE;

    switch (bits)
    {
    case 7:
        newtio.c_cflag |= CS7;
        // printf("7数据位\n");
        break;

    case 8:
        newtio.c_cflag |= CS8;
        // printf("8数据位\n");
        break;
    
    default:
        newtio.c_cflag |= CS8;
        // printf("8数据位\n");
        break;
    }

    switch (parity)
    {
    case 'O':
    case 'o':
        newtio.c_cflag |= (PARENB | PARODD);
        newtio.c_cflag |= INPCK;
        // printf("奇校验\n");
        break;

    case 'E':
    case 'e':
        newtio.c_cflag |= PARENB;
        newtio.c_cflag &= ~PARODD;
        newtio.c_iflag |= INPCK;
        // printf("偶校验\n");
        break;

    case 'N':
    default:
        newtio.c_cflag &= ~PARENB;
        newtio.c_iflag &= ~INPCK;
        // printf("设置为无校验");
        break;
    }

    // 设置波特率
    switch (baudrate)
    {
    case 1200:
        speed = B1200;
        // printf("波特率为1200\n");
        break;
    
    case 1800:
        speed = B1800;
        // printf("波特率为1800\n");
        break;

    case 2400:
        speed = B2400;
        // printf("波特率为2400\n");
        break;

    case 4800:
        speed = B4800;
        // printf("波特率为4800\n");
        break;

    case 9600:
        speed = B9600;
        printf("波特率为9600\n");
        break;

    case 19200:
        speed = B19200;
        // printf("波特率为19200\n");
        break;

    case 38400:
        speed = B38400;
        // printf("波特率为38400\n");
        break;

    case 57600:
        speed = B57600;
        // printf("波特率为57600\n");
        break;

    case 115200:
        speed = B115200;
        // printf("波特率为115200\n");
        break;
    
    case 230400:
        speed = B230400;
        // printf("波特率为230400\n");
        break;

    case 460800:
        speed = B460800;
        // printf("波特率为460800\n");
        break;

    case 500000:
        speed = B500000;
        // printf("波特率为500000\n");
        break;

    case 921600:
        speed = B921600;
        printf("波特率为921600\n");
        break;
    
    default:
        speed = B115200;
        // printf("波特率为115200\n");
        break;
    }

    if (0 > cfsetspeed(&newtio, speed)) {
        // printf("设置波特率失败\n");
        return -1;
    }

    // 设置停止位
    switch (stopbit) {
    case 1:
        newtio.c_cflag &= ~CSTOPB;
        // printf("设置1个停止位\n");
        break;

    case 2:
        newtio.c_cflag |= CSTOPB;
        // printf("设置2个停止位\n");
        break;

    default:
        newtio.c_cflag &= ~CSTOPB;
        // printf("默认设置1个停止位\n");
        break;
    }

    // 将MIN和TIME设置为0
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;

    // 清空缓冲区
    if (0 > tcflush(fd, TCIOFLUSH)) {
        // printf("清空缓冲区失败\n");
        return -1;
    }

    // 写入配置、使配置生效
    if (0 > tcsetattr(fd, TCSANOW, &newtio)) {
        // printf("配置串口失败");
        return -1;
    }

    // time(&end_time);
    // printf("设置波特率使用的时间: %f\n", difftime(end_time, start_time));

    return 0;
}

void init_download_to_screen() {
    if (access(SEND_FILE_NAME, F_OK) == 0) {
        tft_data.clear();
        tftfile.open(SEND_FILE_NAME);
        struct stat tft_stat;
        stat(SEND_FILE_NAME, &tft_stat);
        filesize = tft_stat.st_size;
        std::cout << "文件大小为: " << filesize << std::endl;
        std::ostringstream temp;
        temp << tftfile.rdbuf();
        tft_data = temp.str();
        std::cout << "读取的字符串长度为：" << tft_data.length() << std::endl;

        tft_len = tft_data.length();
        tft_end = tft_buff;
		
		send_finish = false;

        tftfile.close();
    }
}

void download_to_screen() {
    std::cout << "开始数据 " << tft_start << std::endl;
    if (tft_start < tft_len) {
        if (tft_end > tft_len) {
            tft_s = tft_data.substr(tft_start, tft_len - tft_start);
            std::cout << "发送下载数据 == " << tft_start << "/" << filesize <<std::endl;  
            send_cmd_download_data(fd, tft_s);
            // close(copy_fd);
			send_finish = true;
        }
		else
		{
			tft_s = tft_data.substr(tft_start, tft_buff);
			std::cout << tft_s.length() << " 发送下载数据 == " << tft_start << "/" << filesize <<std::endl;  
			tft_start = tft_end;
			tft_end = tft_end + tft_buff;
			send_cmd_download_data(fd, tft_s);
		}
    }
}

void send_cmd_download_data(int fd, std::string data) {
   
    int num = 512;
    int len = data.length();
    int end = num;
    std::string sub_data;
	printf("下载数据: %d\n", len);
    for (int start = 0; start < len; ) {
        if (end > len) {
            sub_data = data.substr(start, len - start);
            write(fd, sub_data.data(), sub_data.length());
            break;
        }
        sub_data = data.substr(start, num);
        write(fd, sub_data.data(), sub_data.length());
        start = end;
        end = end + num;
        usleep(55000);
    }
}

void send_cmd_download(int fd, int filesize) {
    std::string cmd = "whmi-wri " + std::to_string(filesize) + ",TRANSFER_BAUD,0\xff\xff\xff";
    write(fd, cmd.data(), cmd.length());
}