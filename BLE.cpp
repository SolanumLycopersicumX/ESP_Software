#include "mbed.h"
// BLEHandler(int baud_rate, *Serial pointer to hm10)

#define N 10


class BLEHandler {
private:
    Serial* hm10;  // tx, rx, baud

    Ticker clk;  // activates tx and rx functions
    float clk_period;  // period of clk

    bool state;

    char char_r;  // current read char
    char char_w;  // char we are transmitting

    char buffer_r[N];  // all char received (buffer = N chars)
    char buffer_w[N];  // characters to transmit

    typedef enum {TX, RX} actions;
    actions action;


    // ISR to attach to a ticker, updates BLE
    void update() {
        char c;

        switch (action) {
        case (TX):
            if (hm10->writeable()) {
                c = take_from_buffer_w();
                if (c or c != ' ') {
                    char_w = c;
                    //hm10->putc(char_w);  // DISABLED WRITE!!!!
                }
            }
            action = RX;  // constantly switch between TX and RX
            break;
        case (RX):
            if (hm10->readable()) {
                c = hm10->getc();
                if (c != '-') {  // non-null values
                    char_r = c;  // if readible and non-nothing, update read value
                    update_buffer_r();
                }
            }
            action = TX;  // constantly switch between TX and RX
            break;
        }
    }

    void update_buffer_r() {
        if (char_r == '/') {  // if this char, clear buffer
            for (int i = 0; i < N; i++) {
                buffer_r[i] = ' ';
            }
        }
        else {  // otherwise, append to buffer
            for (int i = 0; i < N; i++) {
                if (buffer_r[i] == ' ') {
                    buffer_r[i] = char_r;
                    break;
                }
            }
        }
    }

    // Removes characters from buffer_w, one call at a time
    char take_from_buffer_w() {
        char c;

        for (int y=0; y<N; y++) {
            if (buffer_w[y] != ' ') {
                c = buffer_w[y];
                buffer_w[y] = ' ';
                return c;
            }
        }

        return ' ';
    }


public:
    BLEHandler(int baud_rate, Serial* hm10_ptr) : hm10(hm10_ptr) {  // hm10(ble_tx, ble_rx), 

        // init everything
        baud(baud_rate);
        action = RX;
        char_r = '/'; char_w = '/';  // default values
        update_buffer_r();
        on();
    }

    // Returns the last received character
    char read_last() {
        return char_r;
    }

    // Returns all buffered characters, clear buffer by sending '/'
    char* read_string() {
        return buffer_r;
    }

    float read_double() {
        float f;
        f = (float)atof(buffer_r);
        return f;
    }

    void write_char(char c) {
        char_w = c;
    }

    char write_get_char() {
        return char_w;
    }

    void write_string(char* s) {
        int size = sizeof(&s)/sizeof(s[0]);

        for (int x=0; x<size or x<N; x++) {
            buffer_w[x] = s[x];
        }
    }

    char* write_get_string() {
        return buffer_w;
    }

    // Sets the desired baud_rate of the hm10
    void baud(int baud_rate) {
        hm10->baud(baud_rate);
        clk_period = 9.0f / (4 * baud_rate);
        // 9 bits per ble message (8 bit data + 1 stop bit)
        // f_max = baud/9 bits
        // f_max = 2*baud/9 bits (x2 as it does two things, TX and RX)
        // f_sampling = 2*f_max = 4*baud/9bits
        // clk_period = 1/f_sampling = 9/(4*baud)
    }

    // turns on ticker, to update BLE
    void on() {
        state = true;
        clk.attach(callback(this, &BLEHandler::update), clk_period);
    }

    // turns off ticker, to stop update BLE
    void off() {
        state = false;
        clk.detach();
    }

    void toggle() {
        if (state) {off();}  // if on --> off
        else {on();}         // if off --> on
    }

    // Is the ble enabled or disabled? (1 or 0)
    bool get_state() {return state;}

    // Are we TX or RX? ('t' or 'r')
    char get_action() {
        if (action == RX) {
            return 'r';
        } else if (action == TX) {
            return 't';
        }
        return ' ';
    }
};
