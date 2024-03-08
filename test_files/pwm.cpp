#include <iostream>
#include <fstream>
#include <unistd.h>
#include <thread>

#define GPIO_ROOT "/sys/class/gpio"
#define EXPORT "/sys/class/gpio/export"
#define UNEXPORT "/sys/class/gpio/unexport"

#define LOW 0
#define HIGH 1

class PWM {
private:
    int pin;
    std::thread pwm_thread;
    int duty_cycle;
    bool keep_running = true;
    int frequency;

public:
    PWM(int pin, int initial_duty_cycle, int frequency = 100) : pin(pin), duty_cycle(initial_duty_cycle),frequency{frequency} {
        // Export the pin (Initialize the pin)
        std::ofstream export_file(EXPORT);
        if (!export_file.is_open()) {
            std::cout << "Failed to open the export file" << std::endl;
        }
        export_file << pin;
        export_file.close();

        std::string direction_file_path = std::string(GPIO_ROOT) + "/gpio" + std::to_string(pin) + "/direction";
        std::ofstream direction_file(direction_file_path);
        if (!direction_file.is_open()) {
            std::cout << "Failed to open the direction file" << std::endl;
        }
        direction_file << "out";
        direction_file.close();

        // Start the PWM thread
        pwm_thread = std::thread(&PWM::changeDutyCycle, this);
    }

    ~PWM() {

        this->keep_running = false;
        // Join the PWM thread
        if (pwm_thread.joinable()) {
            pwm_thread.join();
        }
        // Unexport the pin (Release the pin)
        std::ofstream unexport_file(UNEXPORT);
        if (!unexport_file.is_open()) {
            std::cout << "Failed to open the unexport file" << std::endl;
        }
        unexport_file << pin;
        unexport_file.close();

        std::cout<<"Destroyed"<<std::endl;
        
    }

    void setValue(int val) {
        std::string value_file_path = std::string(GPIO_ROOT) + "/gpio" + std::to_string(pin) + "/value";
        std::ofstream value_file(value_file_path);
        if (!value_file.is_open()) {
            std::cout << "Failed to open the value file" << std::endl;
        }
        value_file << val;
        value_file.close();
    }

    void changeDutyCycle() {
        int period = 1000000 / this->frequency; // Period in microseconds

        while (this->keep_running) {
            int onTime = (duty_cycle * period) / 100; // On time in microseconds
            int offTime = period - onTime; // Off time in microseconds

            this->setValue(HIGH);
            usleep(onTime);
            this->setValue(LOW);
            usleep(offTime);
        }
    }

    void setDutyCycle(int new_duty_cycle) {
        duty_cycle = new_duty_cycle;
    }
};

int main() {
    

    PWM pin1(25, 100,100);
    PWM pin2(12, 0,100);

    std::cout << "PWM started" << std::endl;

    for(int i = 0; i < 100; i+=5) {
        pin1.setDutyCycle(100 - i);
        pin2.setDutyCycle(i);
        sleep(1);
    }

    return 0;
}
