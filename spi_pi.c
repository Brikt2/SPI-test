
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <time.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <asm/termbits.h>
#include <ws2811.h>

#define LED_PIN 18
#define ANTALL_PIKSLER 60
#define DMX_CHANNELS 512
#define UART_DEVICE "/dev/serial0"

static volatile sig_atomic_t running = 1;

static ws2811_t ledstring = {
    .freq = WS2811_TARGET_FREQ,
    .dmanum = 10,
    .channel = {
        [0] = {
            .gpionum = LED_PIN,
            .count = ANTALL_PIKSLER,
            .invert = 0,
            .brightness = 60,
            .strip_type = WS2811_STRIP_GRB,
        },
    },
};

enum MarkerState {
    MARK_NORMAL,
    MARK_AFTER_FF,
    MARK_AFTER_FF_00
};

enum DmxState {
    WAIT_BREAK,
    WAIT_STARTCODE,
    READ_CHANNELS
};

static void handle_signal(int signal)
{
    (void)signal;
    running = 0;
}

static int setup_uart(int fd)
{
    struct termios2 options;

    if (ioctl(fd, TCGETS2, &options) < 0) {
        perror("TCGETS2");
        return -1;
    }

    // DMX: 250000 baud, 8 databiter, ingen paritet, 2 stoppbiter
    options.c_cflag &= ~(CBAUD | CSIZE | PARENB | PARODD | CSTOPB);
    options.c_cflag |= BOTHER | CS8 | CSTOPB | CLOCAL | CREAD;

    options.c_ispeed = 250000;
    options.c_ospeed = 250000;

    // Behold BREAK og framing-feil som PARMRK-sekvenser
    options.c_iflag = INPCK | PARMRK;
    options.c_oflag = 0;
    options.c_lflag = 0;

    options.c_cc[VMIN] = 1;
    options.c_cc[VTIME] = 0;

    if (ioctl(fd, TCSETS2, &options) < 0) {
        perror("TCSETS2");
        return -1;
    }

    return 0;
}

static void update_leds(const uint8_t dmx[])
{
    for (int i = 0; i < ANTALL_PIKSLER; i++) {

        int kanal = i * 3;

        uint8_t red   = dmx[kanal];
        uint8_t green = dmx[kanal + 1];
        uint8_t blue  = dmx[kanal + 2];

        uint32_t farge =
            ((uint32_t)red << 16) |
            ((uint32_t)green << 8) |
            (uint32_t)blue;

        ledstring.channel[0].leds[i] = farge;
    }

    ws2811_return_t ret = ws2811_render(&ledstring);

    if (ret != WS2811_SUCCESS) {
        fprintf(stderr, "LED-oppdatering feilet: %s\n",
                ws2811_get_return_t_str(ret));
    }
}

int main(void)
{
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    // Initialiser LED-stripen
    ws2811_return_t ret = ws2811_init(&ledstring);

    if (ret != WS2811_SUCCESS) {
        fprintf(stderr, "LED-init feilet: %s\n",
                ws2811_get_return_t_str(ret));
        return 1;
    }

    // Start med alle LED-er avslått
    for (int i = 0; i < ANTALL_PIKSLER; i++) {
        ledstring.channel[0].leds[i] = 0;
    }

    ws2811_render(&ledstring);

    // Åpne UART
    int serial = open(UART_DEVICE, O_RDONLY | O_NOCTTY);

    if (serial < 0) {
        perror("Kunne ikke åpne UART");
        ws2811_fini(&ledstring);
        return 1;
    }

    if (setup_uart(serial) < 0) {
        close(serial);
        ws2811_fini(&ledstring);
        return 1;
    }

    uint8_t dmx[DMX_CHANNELS] = {0};

    enum MarkerState marker_state = MARK_NORMAL;
    enum DmxState dmx_state = WAIT_BREAK;

    int channel = 0;

    unsigned long break_count = 0;
    unsigned long frame_count = 0;
    unsigned long error_count = 0;

    time_t last_report = time(NULL);

    printf("DMX til LED startet.\n");
    printf("UART: %s, 250000 baud, 8N2\n", UART_DEVICE);
    printf("Venter på DMX...\n");
    fflush(stdout);

    uint8_t buffer[256];

    while (running) {

        ssize_t n = read(serial, buffer, sizeof(buffer));

        if (n < 0) {
            if (errno == EINTR)
                continue;

            perror("UART read");
            break;
        }

        if (n == 0)
            continue;

        for (ssize_t j = 0; j < n; j++) {

            uint8_t byte = buffer[j];

            // Dekod Linux PARMRK
            int event = 0;
            uint8_t value = 0;

            switch (marker_state) {

                case MARK_NORMAL:
                    if (byte == 0xFF) {
                        marker_state = MARK_AFTER_FF;
                    } else {
                        event = 1;
                        value = byte;
                    }
                    break;

                case MARK_AFTER_FF:
                    if (byte == 0xFF) {
                        // Vanlig dataverdi 255
                        marker_state = MARK_NORMAL;
                        event = 1;
                        value = 0xFF;
                    } else if (byte == 0x00) {
                        marker_state = MARK_AFTER_FF_00;
                    } else {
                        marker_state = MARK_NORMAL;
                        event = 3;
                    }
                    break;

                case MARK_AFTER_FF_00:
                    marker_state = MARK_NORMAL;

                    if (byte == 0x00) {
                        // BREAK registrert
                        event = 2;
                    } else {
                        // Paritets- eller framing-feil
                        event = 3;
                    }
                    break;
            }

            // Behandle hendelsen
            if (event == 2) {

                break_count++;

                // En komplett nok pakke har kommet inn.
                // Oppdater LED-ene ved neste BREAK.
                if (dmx_state == READ_CHANNELS &&
                    channel >= ANTALL_PIKSLER * 3) {

                    update_leds(dmx);
                    frame_count++;
                }

                channel = 0;
                dmx_state = WAIT_STARTCODE;
            }

            else if (event == 3) {

                error_count++;
                channel = 0;
                dmx_state = WAIT_BREAK;
            }

            else if (event == 1) {

                switch (dmx_state) {

                    case WAIT_BREAK:
                        // Vent på neste BREAK
                        break;

                    case WAIT_STARTCODE:
                        if (value == 0x00) {
                            // Standard DMX-startkode
                            channel = 0;
                            dmx_state = READ_CHANNELS;
                        } else {
                            // Ikke-standard startkode ignoreres
                            dmx_state = WAIT_BREAK;
                        }
                        break;

                    case READ_CHANNELS:
                        if (channel < DMX_CHANNELS) {
                            dmx[channel++] = value;
                        }
                        break;
                }
            }
        }

        // Statusmelding omtrent én gang i sekundet
        time_t now = time(NULL);

        if (now != last_report) {

            printf("BREAK: %lu | DMX-pakker: %lu | Feil: %lu",
                   break_count, frame_count, error_count);

            if (frame_count > 0) {
                printf(" | LED 1 RGB: %u, %u, %u",
                       dmx[0], dmx[1], dmx[2]);
            }

            printf("\n");
            fflush(stdout);

            last_report = now;
        }
    }

    printf("\nAvslutter...\n");

    close(serial);
    ws2811_fini(&ledstring);

    return 0;
}
