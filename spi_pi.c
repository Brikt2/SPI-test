
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <signal.h>
#include <ws2811.h>

#define LED_PIN 18
#define ANTALL_PIKSLER 60

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

int main(void)
{
    ws2811_return_t ret;

    ret = ws2811_init(&ledstring);

    if (ret != WS2811_SUCCESS) {
        fprintf(stderr, "Funka ikke %s\n",
                ws2811_get_return_t_str(ret));
        return 1;
    }


    while (1) {
        uint8_t red = 255;
        uint8_t green = 0;
        uint8_t blue = 0;
        uint32_t farge = ((uint32_t)red << 16) | ((uint32_t)green << 8) | blue;
        ledstring.channel[0].leds[0] = farge;

        for (int i = 0; i < ANTALL_PIKSLER; i++) {
            ledstring.channel[0].leds[i] = farge;
        }

        ws2811_render(&ledstring);
        usleep(100000);

    }

    ws2811_fini(&ledstring);
    return 0;
}
