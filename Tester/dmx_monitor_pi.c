
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <asm/termbits.h>
#include <time.h>

#define DMX_CHANNELS 512
#define LED_COUNT 60

enum {
    WAIT_BREAK,
    WAIT_STARTCODE,
    READ_CHANNELS,
    AFTER_FF,
    AFTER_FF_ZERO
};

void print_dmx(const uint8_t dmx[], int channels)
{
    printf("\n--- DMX DATA ---\n");

    int leds = channels / 3;

    if (leds > LED_COUNT)
        leds = LED_COUNT;

    for (int i = 0; i < leds; i++) {
        int kanal = i * 3;

        printf("LED %02d: R=%3u G=%3u B=%3u\n",
               i + 1,
               dmx[kanal],
               dmx[kanal + 1],
               dmx[kanal + 2]);
    }

    fflush(stdout);
}

int main(void)
{
    int serial = open("/dev/serial0", O_RDONLY | O_NOCTTY);

    if (serial < 0) {
        perror("Kunne ikke åpne seriellport");
        return 1;
    }

    struct termios2 options;

    if (ioctl(serial, TCGETS2, &options) < 0) {
        perror("TCGETS2");
        close(serial);
        return 1;
    }

    // DMX: 250000 baud, 8 databiter, ingen paritet, 2 stoppbiter
    options.c_cflag &= ~(CBAUD | CSIZE | PARENB);
    options.c_cflag |= BOTHER | CS8 | CSTOPB | CLOCAL | CREAD;

    options.c_ispeed = 250000;
    options.c_ospeed = 250000;

    // Rapporter BREAK og byte-feil
    options.c_iflag = INPCK | PARMRK;
    options.c_oflag = 0;
    options.c_lflag = 0;

    options.c_cc[VMIN] = 1;
    options.c_cc[VTIME] = 0;

    if (ioctl(serial, TCSETS2, &options) < 0) {
        perror("TCSETS2");
        close(serial);
        return 1;
    }

    uint8_t dmx[DMX_CHANNELS] = {0};

    int state = WAIT_BREAK;
    int channel = 0;
    int frame_valid = 0;

    struct timespec last_print = {0, 0};

    printf("DMX-monitor startet.\n");
    printf("Venter på DMX...\n");

    while (1) {
        uint8_t byte;

        int n = read(serial, &byte, 1);

        if (n < 0) {
            perror("read");
            break;
        }

        if (n == 0)
            continue;

        switch (state) {

            case WAIT_BREAK:
                if (byte == 0xFF)
                    state = AFTER_FF;
                break;

            case WAIT_STARTCODE:
                if (byte == 0x00) {
                    // Standard DMX startkode
                    channel = 0;
                    frame_valid = 1;
                    state = READ_CHANNELS;
                } else {
                    // Ukjent startkode, ignorer denne pakken
                    state = WAIT_BREAK;
                }
                break;

            case READ_CHANNELS:
                if (byte == 0xFF) {
                    state = AFTER_FF;
                } else if (channel < DMX_CHANNELS) {
                    dmx[channel++] = byte;
                }
                break;

            case AFTER_FF:
                if (byte == 0x00) {
                    state = AFTER_FF_ZERO;
                } else if (byte == 0xFF) {
                    // Vanlig dataverdi 255
                    if (state == AFTER_FF && frame_valid &&
                        channel < DMX_CHANNELS) {
                        dmx[channel++] = 0xFF;
                    }
                    state = READ_CHANNELS;
                } else {
                    state = WAIT_BREAK;
                    frame_valid = 0;
                }
                break;

            case AFTER_FF_ZERO:
                if (byte == 0x00) {
                    // BREAK registrert
                    if (frame_valid && channel > 0) {
                        struct timespec now;
                        clock_gettime(CLOCK_MONOTONIC, &now);

                        if (now.tv_sec - last_print.tv_sec >= 1) {
                            print_dmx(dmx, channel);
                            last_print = now;
                        }
                    }

                    frame_valid = 0;
                    channel = 0;
                    state = WAIT_STARTCODE;
                } else {
                    // UART rapporterte parity-/framing-feil
                    frame_valid = 0;
                    channel = 0;
                    state = WAIT_BREAK;
                }
                break;
        }
    }

    close(serial);
    return 0;
}
