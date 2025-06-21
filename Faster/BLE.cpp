#include "mbed.h"
// BLEHandler(int baud_rate, *Serial pointer to hm10)


#define N 100  // Size of strings that can be read, can be changed.
#define N_w 20  // Size of strings that can be written - THIS IS DEFINED BY HARDWARE


class BLEHandler {
private:
    Serial* hm10;  // tx, rx, baud

    Ticker clk;  // activates tx and rx functions
    float clk_period;  // period of clk

    bool state;

    bool done_w;  // is it ready to write a new message?

    char char_r;  // current read char

    char buffer_r[N];  // all char received (buffer = N chars)
    char buffer_w[N_w];  // characters to transmit

    typedef enum {TX, RX} actions;
    actions action;


    // ISR to attach to a ticker, updates BLE
    void update() {
        char c;

        switch (action) {
        case (TX):
            if (hm10->writeable() && !done_w) {
                hm10->printf(buffer_w);
                done_w = true;
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
            clear_buffer_r();
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

    void clear_buffer_r() {
        for (int i = 0; i < N; i++) {
            buffer_r[i] = ' ';
        }
    }

    void clear_buffer_w() {
        for (int i = 0; i < N_w; i++) {
            buffer_w[i] = 0;
        }
    }


public:
    BLEHandler(int baud_rate, Serial* hm10_ptr) : hm10(hm10_ptr) {  // hm10(ble_tx, ble_rx), 

        // init everything
        baud(baud_rate);
        action = RX; done_w = true;
        char_r = '/';  // default values
        update_buffer_r();
        on();
    }

    // Returns the last received character
    char read_char() {
        return char_r;
    }

    void read_char_set() {
        char_r = '/';
    }

    // Returns all buffered characters, clear buffer by sending '/'
    char* read_string() {
        return buffer_r;
    }

    float read_float() {
        float f;
        f = (float)atof(buffer_r);
        return f;
    }

    void write_char(char c) {
        char char_as_string[N] = {c};
        write_string(char_as_string);
    }

    void write_float(float f, int figures = 3) {
        char s[figures];
        snprintf(s, N, "%f", f);
        write_string(s);
    }

    void write_float_array(float* fa, int size, int figures = 3) {
        char s_fa[figures+1];
        char s[N] = {};
        char space[1] = {' '};

        // for every float in fa
        for (int j=0; j < size; j++) {
            snprintf(s_fa, figures + 1, "%f", fa[j]);  // turn float into char[]
            strcat(s, s_fa);  // s += s_fa;
            strcat(s, space);  // s += ' ';
        }
        write_string(s);  // send string to write
    }

    void write_string(char* s) {
        int size = sizeof(&s)/sizeof(s[0]);
        clear_buffer_w();  // empty buffer w

        if (done_w) {  // if nothing is being written, write as ordered
            done_w = false;  // declare business

            // copy s to buffer_w
            for (int x=0; x<size or x<N_w; x++) {
                buffer_w[x] = s[x];
            }
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
