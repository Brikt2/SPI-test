#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>

#define DMX_CHANNELS 512

int main(void)
{
    int serial = open("/dev/serial0", O_RDONLY | O_NOCTTY);

    if (serial < 0) {
        perror("Kunne ikke åpne seriellport");
        return 1;
    }

    struct termios options;
    tcgetattr(serial, &options);

    cfmakeraw(&options);

    cfsetispeed(&options, B250000);
    cfsetospeed(&options, B250000);

    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8 | CLOCAL | CREAD;
    options.c_cflag |= CSTOPB;  // 2 stoppbiter

    options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP);
    options.c_iflag |= PARMRK;

    options.c_cc[VMIN] = 1;
    options.c_cc[VTIME] = 0;

    tcsetattr(serial, TCSANOW, &options);

    uint8_t dmx[DMX_CHANNELS];
    int channel = 0;
    int state = 0;

    printf("Venter på DMX...\n");

    while (1) {
        uint8_t byte;
        int n = read(serial, &byte, 1);

        if (n <= 0)
            continue;

        if (state == 0) {
            if (byte == 0xFF)
                state = 1;
        }
        else if (state == 1) {
            if (byte == 0x00)
                state = 2;
            else
                state = 0;
        }
        else if (state == 2) {
            if (byte == 0x00) {
                // BREAK registrert
                channel = -1;
                state = 0;
            }
            else if (byte == 0xFF) {
                // Vanlig databyte med verdien 255
                state = 0;
            }
            else {
                state = 0;
            }
        }

        if (channel == -1) {
            // Første byte etter BREAK er startkode
            if (byte == 0) {
                channel = 0;
            } else {
                channel = -2;
            }
        }
        else if (channel >= 0) {
            dmx[channel] = byte;
            channel++;

            if (channel == DMX_CHANNELS) {
                printf("LED 1: R=%3d G=%3d B=%3d | ",
                       dmx[0], dmx[1], dmx[2]);

                printf("LED 60: R=%3d G=%3d B=%3d\n",
                       dmx[177], dmx[178], dmx[179]);

                channel = -2;
            }
        }
    }

    close(serial);
    return 0;
}