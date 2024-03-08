#include <iostream>
#include <fstream>

int main() {
    std::ifstream usb("/dev/ttyUSB0"); // Replace "/dev/ttyUSB0" with the appropriate USB port

    if (!usb.is_open()) {
        std::cout << "Failed to open USB port." << std::endl;
        return 1;
    }

    std::string data;
    while (std::getline(usb, data)) {
        std::cout << "Received data: " << data << std::endl;
    }

    usb.close();

    return 0;
}
