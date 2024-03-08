#include <iostream>
#include <fstream>
#include <unistd.h>

#define GPIO_ROOT "/sys/class/gpio"
#define EXPORT "/sys/class/gpio/export"
#define UNEXPORT "/sys/class/gpio/unexport"

#define LOW 0
#define HIGH 1

class GPIO {
private:
    int pin;

public:
    GPIO(int pin) : pin(pin) {

        // Export the pin (Initilaize the pin)
        std::ofstream export_file(EXPORT);
        if (!export_file.is_open()) {
            std::cout << "Failed to open the export file" << std::endl;
        }
        export_file << pin;
        export_file.close();

    }

    ~GPIO() {
        // Unexport the pin (Release the pin)
        std::ofstream unexport_file(UNEXPORT);
        if (!unexport_file.is_open()) {
            std::cout << "Failed to open the unexport file" << std::endl;
        }
        unexport_file << pin;
        unexport_file.close();
    }


    void set_as(const std::string& mode) {
        std::string direction_file_path = std::string(GPIO_ROOT) + "/gpio" + std::to_string(pin) + "/direction";
        std::ofstream direction_file(direction_file_path);
        if (!direction_file.is_open()) {
            std::cout << "Failed to open the direction file" << std::endl;
        }
        direction_file << mode;
        direction_file.close();
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

    void changeDutyCycle(int dutyCycle) {

        int period = 1000000 / 20; // Period in microseconds
        int onTime = (dutyCycle * period) / 100; // On time in microseconds
        int offTime = period - onTime; // Off time in microseconds

        while (true) {
            this->setValue(HIGH);
            usleep(onTime);
            this->setValue(LOW);
            usleep(offTime);
        }
    }

};
    






int main() {
    int num1 = 12;
    int num2 = 25;
    int num_blinks = 5;
    int result;

    GPIO pin1(num1);
    pin1.set_as("out");

    GPIO pin2(num2);
    pin2.set_as("out");

    for (int i = 0; i < num_blinks; i++) {
        pin1.setValue(HIGH);
        pin2.setValue(LOW);
        sleep(1);
        pin1.setValue(LOW);
        pin2.setValue(HIGH);
        sleep(1);
    }
    pin1.changeDutyCycle(10);
    return 0;
}