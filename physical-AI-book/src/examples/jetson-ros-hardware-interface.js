/**
 * ROS 2 Humble Example: Jetson Hardware Interface Node
 *
 * This example demonstrates how to interface with Jetson hardware components
 * like GPIO, I2C sensors, and camera modules using ROS 2 Humble.
 */

const rclnodejs = require('rclnodejs');
const { QoS } = rclnodejs;

// Mock implementations for Jetson hardware interfaces
// In a real implementation, these would interface with actual Jetson hardware
class JetsonHardwareInterface {
  constructor() {
    this.gpio = {
      setMode: (pin, mode) => console.log(`Setting GPIO pin ${pin} to ${mode}`),
      write: (pin, value) => console.log(`Writing ${value} to GPIO pin ${pin}`),
      read: (pin) => Math.random() > 0.5 ? 1 : 0 // Simulated read
    };

    this.i2c = {
      open: (bus, address) => console.log(`Opening I2C device at address 0x${address.toString(16)}`),
      readByte: (address, register) => Math.floor(Math.random() * 255), // Simulated sensor reading
      writeByte: (address, register, value) => console.log(`Writing ${value} to I2C device 0x${address.toString(16)} register 0x${register.toString(16)}`)
    };

    this.camera = {
      initialize: () => console.log('Initializing Jetson camera interface'),
      capture: () => ({ width: 640, height: 480, data: new Uint8Array(640 * 480 * 3) }) // Simulated frame
    };
  }
}

async function main() {
  await rclnodejs.init();
  const node = rclnodejs.createNode('jetson_hardware_interface');

  // Create Jetson hardware interface
  const hwInterface = new JetsonHardwareInterface();

  // Initialize camera
  hwInterface.camera.initialize();

  // Create publisher for sensor data
  const sensorPublisher = node.createPublisher(
    {
      type: 'std_msgs/msg/Float32MultiArray',
      descriptor: {
        fields: [
          { name: 'layout', type: { type: 8 }}, // Message
          { name: 'data', type: { type: 7, array: true } } // float32 array
        ]
      }
    },
    'jetson/sensor_data'
  );

  // Create publisher for camera frames
  const imagePublisher = node.createPublisher(
    {
      type: 'sensor_msgs/msg/Image',
      descriptor: {
        fields: [
          { name: 'header', type: { type: 8 }}, // std_msgs/Header
          { name: 'height', type: { type: 4 }}, // uint32
          { name: 'width', type: { type: 4 }}, // uint32
          { name: 'encoding', type: { type: 8 }}, // string
          { name: 'is_bigendian', type: { type: 1 }}, // uint8
          { name: 'step', type: { type: 4 }}, // uint32
          { name: 'data', type: { type: 6, array: true } } // uint8 array
        ]
      }
    },
    'jetson/camera/image_raw'
  );

  // Create service for GPIO control
  const gpioService = node.createService(
    {
      type: 'std_srvs/srv/SetBool',
      descriptor: {
        fields: [
          { name: 'data', type: { type: 1 }}, // bool
          { name: 'success', type: { type: 1 }}, // bool
          { name: 'message', type: { type: 8 }} // string
        ]
      }
    },
    'jetson/gpio_control',
    (request, response) => {
      console.log(`GPIO control request: ${request.data}`);
      hwInterface.gpio.write(7, request.data ? 1 : 0); // Use GPIO pin 7
      response.success = true;
      response.message = `GPIO set to ${request.data ? 'HIGH' : 'LOW'}`;
      return response;
    }
  );

  // Timer to periodically publish sensor data
  let count = 0;
  const timer = node.createTimer(1000, () => { // Publish every second
    // Simulate reading from various sensors connected via I2C
    const imu_data = [
      hwInterface.i2c.readByte(0x68, 0x3B), // Accel X
      hwInterface.i2c.readByte(0x68, 0x3D), // Accel Y
      hwInterface.i2c.readByte(0x68, 0x3F), // Accel Z
      hwInterface.i2c.readByte(0x68, 0x43), // Gyro X
      hwInterface.i2c.readByte(0x68, 0x45), // Gyro Y
      hwInterface.i2c.readByte(0x68, 0x47)  // Gyro Z
    ];

    const sensor_msg = {
      layout: { dim: [], data_offset: 0 },
      data: imu_data
    };

    sensorPublisher.publish(sensor_msg);
    console.log(`Published sensor data: [${imu_data.join(', ')}]`);

    // Publish camera frame
    const frame = hwInterface.camera.capture();
    const image_msg = {
      header: { stamp: node.now().secondsAndNanoseconds, frame_id: 'camera' },
      height: frame.height,
      width: frame.width,
      encoding: 'rgb8',
      is_bigendian: 0,
      step: frame.width * 3,
      data: Array.from(frame.data)
    };

    imagePublisher.publish(image_msg);
    console.log(`Published camera frame: ${frame.width}x${frame.height}`);

    count++;
  });

  console.log('Jetson Hardware Interface Node initialized');
  console.log('Publishing to: jetson/sensor_data, jetson/camera/image_raw');
  console.log('Service available: jetson/gpio_control');

  rclnodejs.spin(node);
}

// Handle graceful shutdown
process.on('SIGINT', () => {
  console.log('\nShutting down Jetson Hardware Interface Node...');
  rclnodejs.shutdown();
  process.exit(0);
});

main().catch(console.error);