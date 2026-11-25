#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>

static bool readLine(int fd, std::string& out) {
    out.clear();
    char c;
    while (true) {
        ssize_t n = read(fd, &c, 1);
        if (n < 0) {
            if (errno == EINTR) continue;
            perror("read");
            return false;
        }
        if (n == 0) return !out.empty();   // timeout
        if (c == '\n') return true;
        if (c != '\r') out += c;
    }
}

int main(int argc, char* argv[]) {
    const char* port = (argc > 1) ? argv[1] : "/dev/ttyACM0";

    int fd = open(port, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        std::cerr << "Cannot open " << port << ": " << strerror(errno) << "\n";
        return 1;
    }

    termios tty{};
    if (tcgetattr(fd, &tty) != 0) { perror("tcgetattr"); return 1; }
    cfmakeraw(&tty);
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 5;    // read timeout: 0.5 s of silence
    if (tcsetattr(fd, TCSANOW, &tty) != 0) { perror("tcsetattr"); return 1; }

    // Opening the port resets the Arduino - wait for it to boot.
    std::cout << "Connecting to " << port << "..." << std::endl;
    sleep(2);
    tcflush(fd, TCIOFLUSH);

    std::cout << "Ready. Type something and press Enter." << std::endl;

    std::string input, reply;
    while (std::cout << "> " << std::flush, std::getline(std::cin, input)) {
        if (input.empty()) continue;

        std::string msg = input + "\n";
        if (write(fd, msg.data(), msg.size()) < 0) { perror("write"); break; }
        tcdrain(fd);

        // Print every line the Arduino sends until it goes quiet.
        bool any = false;
        while (readLine(fd, reply)) {
            std::cout << "Arduino: " << reply << std::endl;
            any = true;
        }
        if (!any) std::cout << "(no reply)" << std::endl;
    }

    close(fd);
    return 0;
}