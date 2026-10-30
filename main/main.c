
#include <stdint.h>
#include <stddef.h>

#include "driver/ledc.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_log.h"

// ==================== НАЛАШТУВАННЯ ====================

#define BUZZER_PIN 2
#define TICK_MS 50

#define PWM_MODE LEDC_LOW_SPEED_MODE
#define PWM_TIMER LEDC_TIMER_0
#define PWM_CHANNEL LEDC_CHANNEL_0
#define PWM_DUTY 512

// Частоти нот (Гц)
#define C4 262
#define D4 294
#define E4 330
#define F4 349
#define G4 392
#define A4 440
#define B4 494
#define C5 523

// Тривалості (1 тік = 50 мс)
#define E 4        // 200 мс
#define Q 8        // 400 мс
#define H 16       // 800 мс
#define W 24       // 1200 мс

#define NOTE_GAP 1       // 50 мс
#define SONG_PAUSE 40    // 2000 мс

#define N(f, t) {f, t}
#define R(t) {0, t}
#define COUNT(a) (sizeof(a) / sizeof((a)[0]))

static const char *TAG = "MUSIC";

// ==================== СТРУКТУРИ ====================

typedef struct {
    uint16_t frequency;
    uint8_t duration;
} note_t;

typedef struct {
    const char *name;
    const note_t *notes;
    size_t count;
} melody_t;


// ==================== 1. BABY SHARK ====================
// Розширена навчальна варіація

static const note_t baby_shark[] = {
    N(C4,E), N(D4,E), N(E4,Q),
    N(C4,E), N(D4,E), N(E4,Q),
    N(C4,E), N(D4,E), N(E4,H),

    R(E),

    N(G4,Q), N(G4,E), N(A4,E),
    N(G4,Q), N(E4,Q),
    N(D4,E), N(C4,H),

    R(E),

    N(C4,E), N(D4,E), N(E4,Q),
    N(C4,E), N(D4,E), N(E4,Q),
    N(C4,E), N(D4,E), N(E4,H),

    R(E),

    N(E4,E), N(E4,E),
    N(D4,Q), N(C4,Q),
    N(D4,E), N(E4,E),
    N(G4,Q), N(E4,Q),
    N(D4,Q), N(C4,H),

    R(E),

    N(C4,E), N(C4,E),
    N(D4,E), N(E4,Q),
    N(G4,Q), N(A4,Q),
    N(G4,Q), N(E4,Q),
    N(D4,Q), N(C4,W)
};


// ==================== 2. JINGLE BELLS ====================
// Розширений приспів

static const note_t jingle_bells[] = {
    N(E4,Q), N(E4,Q), N(E4,H),
    R(E),

    N(E4,Q), N(E4,Q), N(E4,H),
    R(E),

    N(E4,Q), N(G4,Q),
    N(C4,Q), N(D4,Q),
    N(E4,W),

    R(E),

    N(F4,Q), N(F4,Q),
    N(F4,Q), N(F4,Q),
    N(F4,Q), N(E4,Q),
    N(E4,Q), N(E4,E), N(E4,E),
    N(D4,Q), N(D4,Q),
    N(E4,Q), N(D4,H), N(G4,H),

    R(E),

    N(E4,Q), N(E4,Q), N(E4,H),
    R(E),

    N(E4,Q), N(E4,Q), N(E4,H),
    R(E),

    N(E4,Q), N(G4,Q),
    N(C4,Q), N(D4,Q),
    N(E4,W),

    R(E),

    N(F4,Q), N(F4,Q),
    N(F4,Q), N(F4,Q),
    N(F4,Q), N(E4,Q),
    N(E4,Q), N(E4,E), N(E4,E),
    N(G4,Q), N(G4,Q),
    N(F4,Q), N(D4,Q),
    N(C4,W)
};


// ==================== 3. WE WILL ROCK YOU ====================
// Розширена ритмічна варіація

static const note_t we_will_rock_you[] = {
    N(C4,E), N(C4,E), N(D4,H),
    R(E),

    N(C4,E), N(C4,E), N(D4,H),
    R(E),

    N(C4,E), N(C4,E), N(D4,Q),
    N(E4,H),

    R(Q),

    N(G4,Q), N(G4,E),
    N(F4,Q), N(E4,Q),
    N(D4,Q), N(C4,H),

    R(Q),

    N(C4,E), N(C4,E), N(D4,H),
    R(E),

    N(C4,E), N(C4,E), N(D4,H),
    R(E),

    N(C4,E), N(C4,E), N(D4,Q),
    N(E4,H),

    R(Q),

    N(D4,Q), N(D4,Q),
    N(E4,Q), N(G4,Q),
    N(F4,Q), N(E4,Q),
    N(D4,Q), N(C4,H),

    R(Q),

    N(C4,E), N(C4,E), N(D4,H),
    N(C4,E), N(C4,E), N(D4,H),
    N(E4,Q), N(G4,Q),
    N(E4,Q), N(D4,Q),
    N(C4,W)
};


// ==================== 4. TWINKLE TWINKLE ====================
// Повна основна мелодія

static const note_t twinkle[] = {
    // Twinkle twinkle little star
    N(C4,Q), N(C4,Q),
    N(G4,Q), N(G4,Q),
    N(A4,Q), N(A4,Q),
    N(G4,H),

    // How I wonder what you are
    N(F4,Q), N(F4,Q),
    N(E4,Q), N(E4,Q),
    N(D4,Q), N(D4,Q),
    N(C4,H),

    R(E),

    // Up above the world so high
    N(G4,Q), N(G4,Q),
    N(F4,Q), N(F4,Q),
    N(E4,Q), N(E4,Q),
    N(D4,H),

    // Like a diamond in the sky
    N(G4,Q), N(G4,Q),
    N(F4,Q), N(F4,Q),
    N(E4,Q), N(E4,Q),
    N(D4,H),

    R(E),

    // Twinkle twinkle little star
    N(C4,Q), N(C4,Q),
    N(G4,Q), N(G4,Q),
    N(A4,Q), N(A4,Q),
    N(G4,H),

    // How I wonder what you are
    N(F4,Q), N(F4,Q),
    N(E4,Q), N(E4,Q),
    N(D4,Q), N(D4,Q),
    N(C4,W)
};


// ==================== 5. OLD MACDONALD ====================
// Розширена мелодія

static const note_t old_macdonald[] = {
    N(G4,Q), N(G4,Q), N(G4,Q),
    N(D4,Q), N(E4,Q), N(E4,Q),
    N(D4,H),

    N(B4,Q), N(B4,Q),
    N(A4,Q), N(A4,Q),
    N(G4,H),

    R(E),

    N(D4,Q), N(D4,Q),
    N(G4,Q), N(G4,Q), N(G4,Q),
    N(D4,Q), N(E4,Q), N(E4,Q),
    N(D4,H),

    N(B4,Q), N(B4,Q),
    N(A4,Q), N(A4,Q),
    N(G4,H),

    R(E),

    N(D4,Q), N(D4,Q),
    N(G4,Q), N(G4,Q), N(G4,Q),
    N(D4,Q), N(D4,Q),
    N(G4,Q), N(G4,Q), N(G4,H),

    R(E),

    N(G4,E), N(G4,E),
    N(G4,Q), N(G4,Q),
    N(G4,E), N(G4,E),
    N(G4,Q), N(G4,Q),

    R(E),

    N(G4,Q), N(G4,Q), N(G4,Q),
    N(D4,Q), N(E4,Q), N(E4,Q),
    N(D4,H),

    N(B4,Q), N(B4,Q),
    N(A4,Q), N(A4,Q),
    N(G4,W)
};


// ==================== ПЛЕЙЛИСТ ====================

static const melody_t playlist[] = {
    {"Baby Shark", baby_shark, COUNT(baby_shark)},
    {"Jingle Bells", jingle_bells, COUNT(jingle_bells)},
    {"We Will Rock You", we_will_rock_you, COUNT(we_will_rock_you)},
    {"Twinkle Twinkle", twinkle, COUNT(twinkle)},
    {"Old MacDonald", old_macdonald, COUNT(old_macdonald)}
};

#define SONG_COUNT COUNT(playlist)


// ==================== СТАН ПРОГРАВАЧА ====================

typedef enum {
    PLAY_NOTE,
    NOTE_SILENCE,
    BETWEEN_SONGS
} player_state_t;

static player_state_t state = PLAY_NOTE;

static size_t song_index = 0;
static size_t note_index = 0;
static uint16_t ticks_left = 0;


// ==================== ІНІЦІАЛІЗАЦІЯ ====================

static void buzzer_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode = PWM_MODE,
        .timer_num = PWM_TIMER,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = C4,
        .clk_cfg = LEDC_AUTO_CLK
    };

    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    ledc_channel_config_t channel = {
        .gpio_num = BUZZER_PIN,
        .speed_mode = PWM_MODE,
        .channel = PWM_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = PWM_TIMER,
        .duty = 0,
        .hpoint = 0
    };

    ESP_ERROR_CHECK(ledc_channel_config(&channel));
}


// ==================== КЕРУВАННЯ ЗВУКОМ ====================

static void buzzer_stop(void)
{
    ESP_ERROR_CHECK(ledc_set_duty(
        PWM_MODE, PWM_CHANNEL, 0
    ));

    ESP_ERROR_CHECK(ledc_update_duty(
        PWM_MODE, PWM_CHANNEL
    ));
}

static void buzzer_play(uint16_t frequency)
{
    if (frequency == 0) {
        buzzer_stop();
        return;
    }

    ESP_ERROR_CHECK(ledc_set_freq(
        PWM_MODE, PWM_TIMER, frequency
    ));

    ESP_ERROR_CHECK(ledc_set_duty(
        PWM_MODE, PWM_CHANNEL, PWM_DUTY
    ));

    ESP_ERROR_CHECK(ledc_update_duty(
        PWM_MODE, PWM_CHANNEL
    ));
}


// ==================== МУЗИЧНИЙ ТАЙМЕР ====================

static void music_tick(void *arg)
{
    (void)arg;

    const melody_t *song = &playlist[song_index];

    // Початок нового етапу
    if (ticks_left == 0) {

        switch (state) {

            case PLAY_NOTE: {
                const note_t *note = &song->notes[note_index];

                if (note_index == 0) {
                    ESP_LOGI(TAG, "Playing: %s", song->name);
                }

                buzzer_play(note->frequency);
                ticks_left = note->duration;
                break;
            }

            case NOTE_SILENCE:
                buzzer_stop();
                ticks_left = NOTE_GAP;
                break;

            case BETWEEN_SONGS:
                buzzer_stop();
                ESP_LOGI(TAG, "Finished: %s", song->name);
                ticks_left = SONG_PAUSE;
                break;
        }
    }

    // Один тік = 50 мс
    ticks_left--;

    if (ticks_left > 0) {
        return;
    }

    // Визначаємо наступний етап
    switch (state) {

        case PLAY_NOTE:
            state = NOTE_SILENCE;
            break;

        case NOTE_SILENCE:
            note_index++;

            if (note_index >= song->count) {
                state = BETWEEN_SONGS;
            } else {
                state = PLAY_NOTE;
            }
            break;

        case BETWEEN_SONGS:
            song_index = (song_index + 1) % SONG_COUNT;
            note_index = 0;
            state = PLAY_NOTE;
            break;
    }
}


// ==================== APP MAIN ====================

void app_main(void)
{
    buzzer_init();

    esp_timer_handle_t music_timer;

    esp_timer_create_args_t timer_args = {
        .callback = music_tick,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "music_timer"
    };

    ESP_ERROR_CHECK(esp_timer_create(
        &timer_args, &music_timer
    ));

    ESP_ERROR_CHECK(esp_timer_start_periodic(
        music_timer, TICK_MS * 1000
    ));

    ESP_LOGI(TAG, "Music player started on GPIO2");
}
