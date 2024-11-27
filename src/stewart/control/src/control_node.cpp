#include <memory>

#include "control_node.hpp"

ControlNode::ControlNode() : Node("control")
{
  // Opening device
  file = open(device, O_RDWR);
  if (file < 0)
  {
    std::cerr << "Failed to open I2C device" << std::endl;
  }
  initPCA9685(file);

  // Subscribing to normal vector msg callback
  norm_vector_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "/normal_vector", ADVERTISING_FREQ,
      std::bind(
          &ControlNode::controlCallback, this,
          std::placeholders::_1));
}

void ControlNode::controlCallback(
    geometry_msgs::msg::Vector3 msg)
{
  // Determine if Vector is a valid vector
  geometry_msgs::msg::Vector3 validNorm = ControlNode::checkValidNormVector(msg);

  // Calculate the required Corner points coordinates
  geometry_msgs::msg::Vector3 bPoints[4];
  for (int i = 0; i < 4; i++)
    bPoints[i] = ControlNode::bPointIK(i + 1, validNorm);

  // Calcualte the required joint points coordinates
  geometry_msgs::msg::Vector3 pPoints[4];
  for (int i = 0; i < 4; i++)
    pPoints[i] = ControlNode::jointAngleIK(i + 1, bPoints[i]);

  // Calculate each motor required angle
  int theta[4];
  for (int i = 0; i < 4; i++)
    theta[i] = ControlNode::angleTheta(i + 1, pPoints[i]);

  // Find the offsetted angle for each theta
  int offsetThetaPulse[4];
  for (int i = 0; i < 4; i++)
    offsetThetaPulse[i] = ControlNode::offsetThetaPulse(i + 1, theta[i]);

  // Send Commands to motors to move
  for (int i = 0; i < 4; i++)
    setPWM(file, i, 0, offsetThetaPulse[i]);
}

geometry_msgs::msg::Vector3 ControlNode::checkValidNormVector(
    geometry_msgs::msg::Vector3 normVector)
{
  // Extract vector components
  double x = normVector.x;
  double y = normVector.y;
  double z = normVector.z;

  // Check if normVector is a unit vector
  double normVectorLength = pow(x, 2) + pow(y, 2) + pow(z, 2);
  if ((normVectorLength < 1 - NORM_VECTOR_THRESHOLD) || (normVectorLength > 1 + NORM_VECTOR_THRESHOLD))
  {
    // Adjust Vector to be a unit vector
    x = x / normVectorLength;
    y = y / normVectorLength;
    z = z / normVectorLength;
  }

  // Check if normVector is within cone boundry
  // Calculate the radius of the cone boundry
  double boundryRadius = sqrt(1 - pow(NORMAL_Z, 2));
  if ((pow(x, 2) + pow(y, 2)) < boundryRadius)
  {
    // Outside the viable radius of normal vectors, must adjust so its within bounds
    double angle = atan2(y, x);
    x = boundryRadius * cos(angle);
    y = boundryRadius * sin(angle);
    z = NORMAL_Z;
  }
  
  // Output Msg
  geometry_msgs::msg::Vector3 outputNorm;
  outputNorm.x = x;
  outputNorm.y = y;
  outputNorm.z = z;

  return outputNorm;
}

geometry_msgs::msg::Vector3 ControlNode::bPointIK(
    int pointNumber,
    geometry_msgs::msg::Vector3 normV)
{
  double x = 0;
  double y = 0;
  double z = 0;

  double nx = normV.x;
  double ny = normV.y;
  double nz = normV.z;

  switch (pointNumber)
  {
  case 1:
    x = (L * nz) / (sqrt(pow(nz, 2) + pow(nx, 2)));
    z = (L * nx) / (sqrt(pow(nz, 2) + pow(nx, 2))) + HEIGHT;
    break;
  case 2:
    x = -(L * nz) / (sqrt(pow(nz, 2) + pow(nx, 2)));
    z = (L * nx) / -(sqrt(pow(nz, 2) + pow(nx, 2))) + HEIGHT;
    break;
  case 3:
    y = (L * nz) / (sqrt(pow(nz, 2) + pow(ny, 2)));
    z = (L * ny) / (sqrt(pow(nz, 2) + pow(ny, 2))) + HEIGHT;
    break;
  case 4:
    y = -(L * nz) / (sqrt(pow(nz, 2) + pow(ny, 2)));
    z = (L * ny) / -(sqrt(pow(nz, 2) + pow(ny, 2))) + HEIGHT;
    break;
  }

  // Output Msg
  geometry_msgs::msg::Vector3 outputCoordinate;
  outputCoordinate.x = x;
  outputCoordinate.y = y;
  outputCoordinate.z = z;

  return outputCoordinate;
}

geometry_msgs::msg::Vector3 ControlNode::jointAngleIK(
    int jointNumber,
    geometry_msgs::msg::Vector3 bPoint)
{
  double A = 0;
  double B = 0;
  double C = 0;
  double D = 0;
  double E = 0;

  double Px = 0;
  double Py = 0;
  double Pz = 0;

  double bx = bPoint.x;
  double by = bPoint.y;
  double bz = bPoint.z;

  switch (jointNumber)
  {
  case 1:
    A = (bx - l[0]) / bz;
    B = -(pow(bx, 2) + pow(by, 2) + pow(bz, 2) + pow(l[1], 2) - pow(l[0], 2) - pow(l[2], 2)) / (2 * bz);

    C = pow(A, 2) + 1;
    D = 2 * (A * B - l[0]);
    E = pow(B, 2) + pow(l[0], 2) - pow(l[1], 2);

    Px = (-D + sqrt(pow(D, 2) - 4 * C * E)) / (2 * C);
    Py = 0;
    Pz = sqrt(pow(l[1], 2) - pow(Px, 2) + 2 * Px * l[0] - pow(l[0], 2));

    break;
  case 2:
    A = (-bx - l[0]) / bz;
    B = (pow(bx, 2) + pow(by, 2) + pow(bz, 2) + pow(l[1], 2) - pow(l[0], 2) - pow(l[2], 2)) / (2 * bz);

    C = pow(A, 2) + 1;
    D = 2 * (A * B + l[0]);
    E = pow(B, 2) + pow(l[0], 2) - pow(l[1], 2);

    Px = (-D - sqrt(pow(D, 2) - 4 * C * E)) / (2 * C);
    Py = 0;
    Pz = sqrt(pow(l[1], 2) - pow(Px, 2) - 2 * Px * l[0] - pow(l[0], 2));

    break;
  case 3:
    A = (by - l[0]) / bz;
    B = -(pow(bx, 2) + pow(by, 2) + pow(bz, 2) + pow(l[1], 2) - pow(l[0], 2) - pow(l[2], 2)) / (2 * bz);

    C = pow(A, 2) + 1;
    D = 2 * (A * B - l[0]);
    E = pow(B, 2) + pow(l[0], 2) - pow(l[1], 2);

    Px = 0;
    Py = (-D + sqrt(pow(D, 2) - 4 * C * E)) / (2 * C);
    Pz = sqrt(pow(l[1], 2) - pow(Py, 2) + 2 * Py * l[0] - pow(l[0], 2));

    break;
  case 4:
    A = (-by - l[0]) / bz;
    B = (pow(bx, 2) + pow(by, 2) + pow(bz, 2) + pow(l[1], 2) - pow(l[0], 2) - pow(l[2], 2)) / (2 * bz);

    C = pow(A, 2) + 1;
    D = 2 * (A * B + l[0]);
    E = pow(B, 2) + pow(l[0], 2) - pow(l[1], 2);

    Px = 0;
    Py = (-D - sqrt(pow(D, 2) - 4 * C * E)) / (2 * C);
    Pz = sqrt(pow(l[1], 2) - pow(Py, 2) - 2 * Py * l[0] - pow(l[0], 2));

    break;
  }

  // Output Msg
  geometry_msgs::msg::Vector3 outputCoordinate;
  outputCoordinate.x = Px;
  outputCoordinate.y = Py;
  outputCoordinate.z = Pz;

  return outputCoordinate;
}

int ControlNode::angleTheta(
    int motorNumber,
    geometry_msgs::msg::Vector3 pPoint)
{
  double theta = 0;
  double sinTheta = 0;
  double cosTheta = 0;

  double px = pPoint.x;
  double py = pPoint.y;
  double pz = pPoint.z;

  switch (motorNumber)
  {
  case 1:
    sinTheta = (pz / l[1]);
    cosTheta = ((px - l[0]) / (l[1]));

    theta = atan2(sinTheta, cosTheta);
    break;
  case 2:
    sinTheta = (pz / l[1]);
    cosTheta = ((-px - l[0]) / (l[1]));

    theta = atan2(sinTheta, cosTheta);
    break;
  case 3:
    sinTheta = (pz / l[1]);
    cosTheta = ((py - l[0]) / (l[1]));

    theta = atan2(sinTheta, cosTheta);
    break;
  case 4:
    sinTheta = (pz / l[1]);
    cosTheta = ((-py - l[0]) / (l[1]));

    theta = atan2(sinTheta, cosTheta);
    break;
  }

  theta = theta * 180 / M_PI;

  return static_cast<int>(theta);
}

int ControlNode::offsetThetaPulse(
    int motorNumber,
    int theta)
{
  int offsetThetaPulse = 0;
  int offset = 15;
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

int ControlNode::angleToPWM(
    int desiredAngle,
    int angleOffset)
{
  int angle = desiredAngle + angleOffset; // Adjust for Servo's offset

  int pulse_min = 128; // Min pulse length out of 4096
  int pulse_max = 512; // Max pulse length out of 4096
  int pulse = pulse_min + (angle * (pulse_max - pulse_min) / 180);

  // std::cerr << "forward pulse: " << pulse << std::endl;

  return pulse;
}

int ControlNode::angleToPWMReversed(
    int desiredAngle,
    int angleOffset)
{
  int angle = desiredAngle + angleOffset; // Adjust for Servo's offset

  int pulse_min = 512; // Min pulse length out of 4096
  int pulse_max = 128; // Max pulse length out of 4096
  int pulse = pulse_min + (angle * (pulse_max - pulse_min) / 180);

  // std::cerr << "reverse pule: " << pulse << std::endl;

  return pulse;
}

// Write a single byte to a specific register
void ControlNode::writeByte(int file, uint8_t reg, uint8_t value)
{
  uint8_t buffer[2] = {reg, value};
  if (write(file, buffer, 2) != 2)
  {
    std::cerr << "Failed to write to I2C device" << std::endl;
  }
}

// Initialize the PCA9685
void ControlNode::initPCA9685(int file)
{
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
void ControlNode::setPWM(int file, int channel, int on, int off)
{
  int reg_base = LED0_ON_L + 4 * channel;

  writeByte(file, reg_base, on & 0xFF);      // ON low byte
  writeByte(file, reg_base + 1, on >> 8);    // ON high byte
  writeByte(file, reg_base + 2, off & 0xFF); // OFF low byte
  writeByte(file, reg_base + 3, off >> 8);   // OFF high byte
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
