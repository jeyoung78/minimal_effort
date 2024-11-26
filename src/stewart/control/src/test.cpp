#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cmath>
#include <vector>

float bs[3] = {0, 0, 0};
float ls[3] = {sqrt(2 * pow(0.12, 2)), 0.0635, 0.0926};
float P[3] = {0, 0, 0};
float L = sqrt(2 * pow(0.15, 2));

int angleToPWM(int desiredAngle, int angleOffset)
{
    int angle = desiredAngle + angleOffset; // Adjust for Servo's offset

    int pulse_min = 128; // Min pulse length out of 4096
    int pulse_max = 512; // Max pulse length out of 4096
    int pulse = pulse_min + (angle * (pulse_max - pulse_min) / 180);

    // std::cerr << "forward pulse: " << pulse << std::endl;

    return pulse;
}

int angleToPWMReversed(int desiredAngle, int angleOffset)
{
    int angle = desiredAngle + angleOffset; // Adjust for Servo's offset

    int pulse_min = 512; // Min pulse length out of 4096
    int pulse_max = 128; // Max pulse length out of 4096
    int pulse = pulse_min + (angle * (pulse_max - pulse_min) / 180);

    // std::cerr << "reverse pule: " << pulse << std::endl;

    return pulse;
}

int offsetThetaPulse(int motorNumber, int theta)
{
    int offsetThetaPulse = 0;
    int offset = 0;
    switch (motorNumber)
    {
    case 1:
        offsetThetaPulse = angleToPWM(theta, offset);
        break;
    case 2:
        offsetThetaPulse = angleToPWM(theta, offset);
        break;
    case 3:
        offsetThetaPulse = angleToPWMReversed(theta, offset);
        break;
    case 4:
        offsetThetaPulse = angleToPWMReversed(theta, offset);
        break;
    }

    return offsetThetaPulse;
}

void bPointIK(int pointNumber, float L, float nx, float ny, float nz, float h)
{
    // Inverse Kinematics of the points at the corners

    float x = 0;
    float y = 0;
    float z = 0;

    switch (pointNumber)
    {
    case 1:
        x = (L * nz) / (sqrt(pow(nz, 2) + pow(nx, 2)));
        z = (L * nx) / (sqrt(pow(nz, 2) + pow(nx, 2))) + h;
        break;
    case 2:
        x = -(L * nz) / (sqrt(pow(nz, 2) + pow(nx, 2)));
        z = (L * nx) / -(sqrt(pow(nz, 2) + pow(nx, 2))) + h;
        break;
    case 3:
        y = (L * nz) / (sqrt(pow(nz, 2) + pow(ny, 2)));
        z = (L * ny) / (sqrt(pow(nz, 2) + pow(ny, 2))) + h;
        break;
    case 4:
        y = -(L * nz) / (sqrt(pow(nz, 2) + pow(ny, 2)));
        z = (L * ny) / -(sqrt(pow(nz, 2) + pow(ny, 2))) + h;
        break;
    }

    bs[0] = x;
    bs[1] = y;
    bs[2] = z;
}

void jointAngleIK(int jointNumber, float l[], float b[])
{
    float A = 0;
    float B = 0;
    float C = 0;
    float D = 0;
    float E = 0;

    float Px = 0;
    float Py = 0;
    float Pz = 0;

    switch (jointNumber)
    {
    case 1:
        A = (b[0] - l[0]) / b[2];
        B = -(pow(b[0], 2) + pow(b[1], 2) + pow(b[2], 2) + pow(l[1], 2) - pow(l[0], 2) - pow(l[2], 2)) / (2 * b[2]);

        C = pow(A, 2) + 1;
        D = 2 * (A * B - l[0]);
        E = pow(B, 2) + pow(l[0], 2) - pow(l[1], 2);

        Px = (-D + sqrt(pow(D, 2) - 4 * C * E)) / (2 * C);
        Py = 0;
        Pz = sqrt(pow(l[1], 2) - pow(Px, 2) + 2 * Px * l[0] - pow(l[0], 2));

        break;
    case 2:
        A = (-b[0] - l[0]) / b[2];
        B = (pow(b[0], 2) + pow(b[1], 2) + pow(b[2], 2) + pow(l[1], 2) - pow(l[0], 2) - pow(l[2], 2)) / (2 * b[2]);

        C = pow(A, 2) + 1;
        D = 2 * (A * B + l[0]);
        E = pow(B, 2) + pow(l[0], 2) - pow(l[1], 2);

        Px = (-D - sqrt(pow(D, 2) - 4 * C * E)) / (2 * C);
        Py = 0;
        Pz = sqrt(pow(l[1], 2) - pow(Px, 2) - 2 * Px * l[0] - pow(l[0], 2));

        break;
    case 3:
        A = (b[1] - l[0]) / b[2];
        B = -(pow(b[0], 2) + pow(b[1], 2) + pow(b[2], 2) + pow(l[1], 2) - pow(l[0], 2) - pow(l[2], 2)) / (2 * b[2]);

        C = pow(A, 2) + 1;
        D = 2 * (A * B - l[0]);
        E = pow(B, 2) + pow(l[0], 2) - pow(l[1], 2);

        Px = 0;
        Py = (-D + sqrt(pow(D, 2) - 4 * C * E)) / (2 * C);
        Pz = sqrt(pow(l[1], 2) - pow(Py, 2) + 2 * Py * l[0] - pow(l[0], 2));

        break;
    case 4:
        A = (-b[1] - l[0]) / b[2];
        B = (pow(b[0], 2) + pow(b[1], 2) + pow(b[2], 2) + pow(l[1], 2) - pow(l[0], 2) - pow(l[2], 2)) / (2 * b[2]);

        C = pow(A, 2) + 1;
        D = 2 * (A * B + l[0]);
        E = pow(B, 2) + pow(l[0], 2) - pow(l[1], 2);

        Px = 0;
        Py = (-D - sqrt(pow(D, 2) - 4 * C * E)) / (2 * C);
        Pz = sqrt(pow(l[1], 2) - pow(Py, 2) - 2 * Py * l[0] - pow(l[0], 2));

        break;
    }

    // std::cerr << "Px" << Px << "  " << "Py" << Py << "  " << "Pz" << Pz << std::endl;

    P[0] = Px;
    P[1] = Py;
    P[2] = Pz;
}

int angleTheta(int motorNumber, float P[], float l[])
{
    double theta = 0;
    float sinTheta = 0;
    float cosTheta = 0;

    switch (motorNumber)
    {
    case 1:
        sinTheta = static_cast<double>(P[2] / l[1]);
        cosTheta = static_cast<double>((P[0] - l[0]) / (l[1]));

        theta = atan2(sinTheta, cosTheta);
        break;
    case 2:
        sinTheta = static_cast<double>(P[2] / l[1]);
        cosTheta = static_cast<double>((-P[0] - l[0]) / (l[1]));

        theta = atan2(sinTheta, cosTheta);
        break;
    case 3:
        sinTheta = static_cast<double>(P[2] / l[1]);
        cosTheta = static_cast<double>((P[1] - l[0]) / (l[1]));

        theta = atan2(sinTheta, cosTheta);
        break;
    case 4:
        sinTheta = static_cast<double>(P[2] / l[1]);
        cosTheta = static_cast<double>((-P[1] - l[0]) / (l[1]));

        theta = atan2(sinTheta, cosTheta);
        break;
    }

    theta = theta * 180 / M_PI;

    return static_cast<int>(theta);
}

#define PCA9685_ADDRESS 0x40 // Default I2C address for PCA9685
#define MODE1 0x00
#define PRESCALE 0xFE

// Define registers for each PWM channel
#define LED0_ON_L 0x06
#define LED0_ON_H 0x07
#define LED0_OFF_L 0x08
#define LED0_OFF_H 0x09

// Write a single byte to a specific register
void writeByte(int file, uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    if (write(file, buffer, 2) != 2) {
        std::cerr << "Failed to write to I2C device" << std::endl;
    }
}

// Initialize the PCA9685
void initPCA9685(int file) {
    ioctl(file, I2C_SLAVE, PCA9685_ADDRESS);
    // Set the MODE1 register to 0x10 to sleep
    writeByte(file, MODE1, 0x10);

    // Set the prescale to get a 50Hz frequency (for servos)
    int prescale_value = std::round(25000000.0 / (4096 * 50) - 1);
    writeByte(file, PRESCALE, prescale_value);

    // Wake up the PCA9685
    writeByte(file, MODE1, 0x00);
    usleep(500);
    writeByte(file, MODE1, 0x80); // Auto-increment enabled
}

// Set PWM for a specific channel
void setPWM(int file, int channel, int on, int off) {
    int reg_base = LED0_ON_L + 4 * channel;

    writeByte(file, reg_base, on & 0xFF);        // ON low byte
    writeByte(file, reg_base + 1, on >> 8);      // ON high byte
    writeByte(file, reg_base + 2, off & 0xFF);   // OFF low byte
    writeByte(file, reg_base + 3, off >> 8);     // OFF high byte
}

int main()
{
    // code to move the fucker
    const char *device = "/dev/i2c-1";
    int file = open(device, O_RDWR);
    if (file < 0) {
        std::cerr << "Failed to open I2C device" << std::endl;
        return 1;
    }
    initPCA9685(file);

    int MOTORS = 5;
    int MOTORS_START = 1;
    double HEIGHT = 0.125; // THIS VALUE IS VERY ARBITRARY, BUT DO NOT CHANGE BECAUSE IT MESSES UP OUR IK

    // do not go greater than +-6.5 degrees LOL
    std::vector<std::vector<double>> normals = {
        {0.1132, 0, 0.9936},
        {0, 0.1132, 0.9936},
        {-0.1132, 0, 0.9936},
        {0, -0.1132, 0.9936}
    };
    
    while (true) { // Infinite loop to keep moving back and forth
        for (const auto normal : normals) {
            std::vector<int> pulses;
            for (int num = MOTORS_START; num < MOTORS; num++)
            {
                // std::cerr << "For motor " << num << " ===============================" << std::endl;
                bPointIK(num, L, normal[0], normal[1], normal[2], HEIGHT);
                // std::cerr << "bs[0]" << bs[0] << "  " << "bs[1]" << bs[1] << "  " << "bs[2]" << bs[2] << std::endl;

                jointAngleIK(num, ls, bs);
                // std::cerr << "ls[0]" << ls[0] << "  " << "ls[1]" << ls[1] << "  " << "ls[2]" << ls[2] << std::endl;

                int theta = angleTheta(num, P, ls);
                // std::cerr << "theta: " << theta << std::endl;

                int offsetThPulse = offsetThetaPulse(num, theta);
                // std::cerr << "offset theta: " << offsetThPulse << std::endl;
                pulses.emplace_back(offsetThPulse);
            }

            setPWM(file, 0, 0, pulses[0]);
            setPWM(file, 1, 0, pulses[1]);
            setPWM(file, 2, 0, pulses[2]);
            setPWM(file, 3, 0, pulses[3]);
            usleep(100000); // Wait before changing direction
        }

        setPWM(file, 0, 0, angleToPWM(0, 15));
        setPWM(file, 1, 0, angleToPWM(0, 15));
        setPWM(file, 2, 0, angleToPWMReversed(0, 15));
        setPWM(file, 3, 0, angleToPWMReversed(0, 15));
        usleep(1000000);
    }

    close(file);
    return 0;
}