#include "auto_car/gpio.hpp"


GPIO::GPIO(int pin) : pin(pin) {

    // Export the pin (Initilaize the pin)
    std::ofstream export_file(EXPORT);
    if (!export_file.is_open()) {
        std::cout << "Failed to open the export file" << std::endl;
    }
    export_file << pin;
    export_file.close();
}

GPIO::GPIO(){
    std::cout<<"Created a object"<<std::endl;
}

GPIO::~GPIO() {
    // Unexport the pin (Release the pin)
    std::ofstream unexport_file(UNEXPORT);
    if (!unexport_file.is_open()) {
        std::cout << "Failed to open the unexport file" << std::endl;
    }
    unexport_file << pin;
    unexport_file.close();
}


void GPIO::setMode(const std::string& mode) {
    std::string direction_file_path = std::string(GPIO_ROOT) + "/gpio" + std::to_string(pin) + "/direction";
    std::ofstream direction_file(direction_file_path);
    if (!direction_file.is_open()) {
        std::cout << "Failed to open the direction file" << std::endl;
    }
    direction_file << mode;
    direction_file.close();
}

void GPIO::setValue(int val) {
    std::string value_file_path = std::string(GPIO_ROOT) + "/gpio" + std::to_string(pin) + "/value";
    std::ofstream value_file(value_file_path);
    if (!value_file.is_open()) {
        std::cout << "Failed to open the value file" << std::endl;
    }
    value_file << val;
    value_file.close();
}

