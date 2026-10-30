
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <signal.h>
#include <ws2811.h>

#define LED_PIN 18
#define LED_COUNT 1

static ws2811_t ledstring = {
    .freq = WS2811_TARGET_FREQ,
    .dmanum = 10,
    .channel = {
        [0] = {
            .gpionum = LED_PIN,
            .count = LED_COUNT,
            .invert = 0,
            .brightness = 60,
            .strip_type = WS2811_STRIP_RGB,
        },
    },
};

int main(void)
{
    ws2811_return_t ret;

    ret = ws2811_init(&ledstring);

    if (ret != WS2811_SUCCESS) {
        fprintf(stderr, "Kunne ikke initialisere LED-driver: %s\n",
                ws2811_get_return_t_str(ret));
        return 1;
    }

    while (1) {
        // Rød
        ledstring.channel[0].leds[0] =
            (uint32_t)255 << 16;

        ws2811_render(&ledstring);
        sleep(1);

        // Av
        ledstring.channel[0].leds[0] = 0;

        ws2811_render(&ledstring);
        sleep(1);
    }

    ws2811_fini();
    return 0;
}
