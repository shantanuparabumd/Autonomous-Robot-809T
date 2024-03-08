#include <iostream>
#include <fstream>
#include <unistd.h>

#define GPIO_ROOT "/sys/class/gpio"
#define EXPORT "/sys/class/gpio/export"
#define UNEXPORT "/sys/class/gpio/unexport"

#define LOW 0
#define HIGH 1

int export_pin(int num) {
    std::ofstream export_file(EXPORT);
    if (!export_file.is_open()) {
        std::cout << "Failed to open the export file" << std::endl;
        return -1;
    }
    export_file << num;
    export_file.close();
    return 0;
}

int unexport_pin(int num) {
    std::ofstream unexport_file(UNEXPORT);
    if (!unexport_file.is_open()) {
        std::cout << "Failed to open the unexport file" << std::endl;
        return -1;
    }
    unexport_file << num;
    unexport_file.close();
    return 0;
}

int set_direction(const std::string& mode, int num) {
    std::string direction_file_path = std::string(GPIO_ROOT) + "/gpio" + std::to_string(num) + "/direction";
    std::ofstream direction_file(direction_file_path);
    if (!direction_file.is_open()) {
        std::cout << "Failed to open the direction file" << std::endl;
        return -1;
    }
    direction_file << mode;
    direction_file.close();
    return 0;
}

int set_value(int val, int num) {
    std::string value_file_path = std::string(GPIO_ROOT) + "/gpio" + std::to_string(num) + "/value";
    std::ofstream value_file(value_file_path);
    if (!value_file.is_open()) {
        std::cout << "Failed to open the value file" << std::endl;
        return -1;
    }
    value_file << val;
    value_file.close();
    return 0;

}

int main() {
    int num = 16;
    int num_blinks = 5;
    int result;

    result = export_pin(num);
    sleep(1);
    result = set_direction("out", num);

    for (int i = 0; i < num_blinks; i++) {
        std::cout<<"Hello"<<std::endl;
        result = set_value(HIGH, num);
        sleep(1);
        result = set_value(LOW, num);
        sleep(1);
    }

    result = set_value(LOW, num);
    result = set_direction("in", num);
    result = unexport_pin(num);

    return 0;
}
