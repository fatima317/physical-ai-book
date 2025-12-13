/**
 * ROS 2 Code Example Validation Test
 * Validates ROS 2 code examples for both simulation and hardware compatibility
 */

const fs = require('fs');
const path = require('path');

class ROS2ValidationTest {
  constructor() {
    this.ros2Packages = [
      'rclpy', 'rclcpp', 'std_msgs', 'sensor_msgs', 'geometry_msgs',
      'nav_msgs', 'tf2_msgs', 'builtin_interfaces', 'action_msgs',
      'lifecycle_msgs', 'unique_identifier_msgs', 'trajectory_msgs',
      'control_msgs', 'diagnostic_msgs', 'shape_msgs', 'stereo_msgs'
    ];

    this.hardwareSpecificPackages = [
      'hardware_interface', 'joint_limits', 'combined_robot_hw',
      'ros2_control', 'ros2_controllers', 'realtime_tools'
    ];

    this.simulationSpecificPackages = [
      'gazebo_ros', 'gazebo_msgs', 'gazebo_plugins',
      'ignition_msgs', 'ignition_transport', 'ros_gz'
    ];

    this.validationResults = [];
  }

  /**
   * Validates a ROS 2 Python code example
   * @param {string} code - The ROS 2 Python code to validate
   * @returns {object} Validation result
   */
  validatePythonROS2Code(code) {
    const result = {
      language: 'python',
      ros2Compatible: true,
      simulationCompatible: true,
      hardwareCompatible: true,
      errors: [],
      warnings: [],
      suggestions: [],
      importIssues: [],
      nodeStructure: false,
      lifecycleIssues: []
    };

    // Check for proper ROS 2 imports
    const hasRclpy = code.includes('import rclpy');
    const hasNodeImport = code.includes('from rclpy.node import Node');

    if (!hasRclpy) {
      result.errors.push('Missing rclpy import - ROS 2 Python client library not imported');
      result.ros2Compatible = false;
    }

    if (!hasNodeImport) {
      result.warnings.push('Node class not imported from rclpy.node - consider inheriting from Node for proper ROS 2 nodes');
    }

    // Check for proper node structure
    if (code.includes('class') && (code.includes('Node') || code.includes('rclpy.node'))) {
      result.nodeStructure = true;
    }

    // Check for proper initialization pattern
    if (code.includes('rclpy.init(') && code.includes('rclpy.spin(') && code.includes('rclpy.shutdown(')) {
      // Good pattern detected
    } else {
      result.warnings.push('Consider using standard ROS 2 initialization pattern: rclpy.init(), rclpy.spin(), rclpy.shutdown()');
    }

    // Check for simulation-specific imports
    const simImports = [];
    for (const pkg of this.simulationSpecificPackages) {
      if (code.includes(pkg)) {
        simImports.push(pkg);
      }
    }

    if (simImports.length > 0) {
      result.warnings.push(`Uses simulation-specific packages: ${simImports.join(', ')}. May not work on real hardware.`);
      result.hardwareCompatible = false;
    }

    // Check for hardware-specific imports
    const hwImports = [];
    for (const pkg of this.hardwareSpecificPackages) {
      if (code.includes(pkg)) {
        hwImports.push(pkg);
      }
    }

    if (hwImports.length > 0) {
      result.warnings.push(`Uses hardware-specific packages: ${hwImports.join(', ')}. May not work in simulation.`);
      result.simulationCompatible = false;
    }

    // Check for common issues
    if (code.includes('time.sleep') && !code.includes('rclpy')) {
      result.warnings.push('Using time.sleep() instead of ROS 2 time utilities. Consider using rclpy.Rate or builtin_interfaces.msg.Time.');
    }

    if (code.includes('while True') && !code.includes('rclpy.ok()')) {
      result.warnings.push('Infinite loop without rclpy.ok() check. Use while rclpy.ok(): for proper shutdown handling.');
    }

    // Check for proper resource management
    if (code.includes('create_publisher') || code.includes('create_subscription') || code.includes('create_service') || code.includes('create_client')) {
      // Check if there's a destroy_node call
      if (!code.includes('destroy_node') && !code.includes('del ')) {
        result.warnings.push('Resource creation detected without explicit cleanup. Consider implementing proper resource management.');
      }
    }

    return result;
  }

  /**
   * Validates a ROS 2 C++ code example
   * @param {string} code - The ROS 2 C++ code to validate
   * @returns {object} Validation result
   */
  validateCppROS2Code(code) {
    const result = {
      language: 'cpp',
      ros2Compatible: true,
      simulationCompatible: true,
      hardwareCompatible: true,
      errors: [],
      warnings: [],
      suggestions: [],
      importIssues: [],
      nodeStructure: false,
      lifecycleIssues: []
    };

    // Check for proper ROS 2 includes
    const hasRclcpp = code.includes('#include <rclcpp/rclcpp.hpp>');

    if (!hasRclcpp) {
      result.errors.push('Missing rclcpp/rclcpp.hpp include - ROS 2 C++ client library not included');
      result.ros2Compatible = false;
    }

    // Check for proper node structure
    if (code.includes('class') && code.includes('rclcpp::Node')) {
      result.nodeStructure = true;
    }

    // Check for proper initialization pattern
    if (code.includes('rclcpp::init(') && code.includes('rclcpp::spin(') && code.includes('rclcpp::shutdown(')) {
      // Good pattern detected
    } else {
      result.warnings.push('Consider using standard ROS 2 initialization pattern: rclcpp::init(), rclcpp::spin(), rclcpp::shutdown()');
    }

    // Check for simulation-specific includes
    const simIncludes = [];
    if (code.includes('gazebo_ros')) {
      simIncludes.push('gazebo_ros');
      result.warnings.push('Uses gazebo_ros packages. May not work on real hardware.');
      result.hardwareCompatible = false;
    }

    if (code.includes('hardware_interface')) {
      result.warnings.push('Uses hardware_interface packages. May not work in simulation.');
      result.simulationCompatible = false;
    }

    // Check for proper resource management
    if (code.includes('create_publisher') || code.includes('create_subscription') || code.includes('create_service') || code.includes('create_client')) {
      // Check if there's a proper cleanup
      if (!code.includes('reset()') && !code.includes('shutdown()')) {
        result.warnings.push('Resource creation detected without explicit cleanup. Consider implementing proper resource management.');
      }
    }

    return result;
  }

  /**
   * Validates ROS 2 code for both simulation and hardware compatibility
   * @param {string} code - The code to validate
   * @param {string} language - The programming language ('python' or 'cpp')
   * @returns {object} Comprehensive validation result
   */
  validateROS2Compatibility(code, language = 'python') {
    let result;

    if (language.toLowerCase() === 'python') {
      result = this.validatePythonROS2Code(code);
    } else if (language.toLowerCase() === 'cpp') {
      result = this.validateCppROS2Code(code);
    } else {
      return {
        error: `Unsupported language: ${language}. Only 'python' and 'cpp' are supported.`
      };
    }

    // Calculate compatibility scores
    result.ros2Score = result.errors.length === 0 ? 100 : Math.max(0, 100 - (result.errors.length * 25));
    result.simulationScore = result.simulationCompatible ? 100 : 60;
    result.hardwareScore = result.hardwareCompatible ? 100 : 60;

    result.overallScore = (result.ros2Score + result.simulationScore + result.hardwareScore) / 3;

    result.passed = result.ros2Score >= 70 && result.overallScore >= 70;

    this.validationResults.push(result);
    return result;
  }

  /**
   * Validates a complete ROS 2 package structure
   * @param {string} packagePath - Path to the ROS 2 package
   * @returns {object} Package validation result
   */
  validateROSPackage(packagePath) {
    const result = {
      packagePath,
      valid: false,
      structure: {
        packageXml: false,
        cmakeLists: false,
        srcDir: false,
        includeDir: false,
        launchDir: false,
        configDir: false
      },
      nodes: [],
      dependencies: [],
      validationResults: []
    };

    try {
      // Check for package.xml
      const packageXmlPath = path.join(packagePath, 'package.xml');
      result.structure.packageXml = fs.existsSync(packageXmlPath);

      if (result.structure.packageXml) {
        const packageXml = fs.readFileSync(packageXmlPath, 'utf8');
        // Basic validation of package.xml content
        if (packageXml.includes('<package format="2">') || packageXml.includes('<package format="3">')) {
          result.valid = true;
        }
      }

      // Check for CMakeLists.txt
      const cmakeListsPath = path.join(packagePath, 'CMakeLists.txt');
      result.structure.cmakeLists = fs.existsSync(cmakeListsPath);

      // Check for source directory
      const srcDirPath = path.join(packagePath, 'src');
      result.structure.srcDir = fs.existsSync(srcDirPath);

      // Check for include directory
      const includeDirPath = path.join(packagePath, 'include');
      result.structure.includeDir = fs.existsSync(includeDirPath);

      // Check for launch directory
      const launchDirPath = path.join(packagePath, 'launch');
      result.structure.launchDir = fs.existsSync(launchDirPath);

      // Check for config directory
      const configDirPath = path.join(packagePath, 'config');
      result.structure.configDir = fs.existsSync(configDirPath);

      // Validate source files
      if (result.structure.srcDir) {
        const srcFiles = fs.readdirSync(srcDirPath);
        for (const file of srcFiles) {
          if (file.endsWith('.py') || file.endsWith('.cpp')) {
            const filePath = path.join(srcDirPath, file);
            const code = fs.readFileSync(filePath, 'utf8');
            const lang = file.endsWith('.py') ? 'python' : 'cpp';
            const validation = this.validateROS2Compatibility(code, lang);
            result.validationResults.push({
              file: file,
              path: filePath,
              validation: validation
            });
          }
        }
      }

      // Validate launch files
      if (result.structure.launchDir) {
        const launchFiles = fs.readdirSync(launchDirPath);
        for (const file of launchFiles) {
          if (file.endsWith('.py') || file.endsWith('.launch.xml')) {
            const filePath = path.join(launchDirPath, file);
            if (file.endsWith('.py')) {
              const code = fs.readFileSync(filePath, 'utf8');
              const validation = this.validateROS2Compatibility(code, 'python');
              result.validationResults.push({
                file: file,
                path: filePath,
                validation: validation
              });
            }
          }
        }
      }

    } catch (error) {
      result.error = error.message;
      result.valid = false;
    }

    return result;
  }

  /**
   * Runs validation on multiple code examples
   * @param {array} codeExamples - Array of code examples to validate
   * @returns {array} Validation results
   */
  runBatchValidation(codeExamples) {
    const results = [];

    for (const example of codeExamples) {
      const result = this.validateROS2Compatibility(
        example.code,
        example.language || 'python'
      );
      results.push({
        name: example.name || 'unnamed',
        code: example.code.substring(0, 100) + '...',
        validation: result
      });
    }

    return results;
  }

  /**
   * Generates a comprehensive validation report
   * @param {array} results - Validation results to include in report
   * @returns {object} Validation report
   */
  generateValidationReport(results = this.validationResults) {
    const totalTests = results.length;
    const passedTests = results.filter(r => r.passed).length;
    const failedTests = totalTests - passedTests;

    const report = {
      timestamp: new Date().toISOString(),
      testName: 'ROS 2 Code Example Validation',
      totalTests,
      passedTests,
      failedTests,
      passRate: totalTests > 0 ? (passedTests / totalTests) * 100 : 0,
      averageScore: results.reduce((sum, r) => sum + r.overallScore, 0) / totalTests || 0,
      compatibilityBreakdown: {
        ros2Compatible: results.filter(r => r.ros2Compatible).length,
        simulationCompatible: results.filter(r => r.simulationCompatible).length,
        hardwareCompatible: results.filter(r => r.hardwareCompatible).length
      },
      summary: {
        errors: results.reduce((sum, r) => sum + r.errors.length, 0),
        warnings: results.reduce((sum, r) => sum + r.warnings.length, 0),
        suggestions: results.reduce((sum, r) => sum + r.suggestions.length, 0)
      },
      detailedResults: results
    };

    return report;
  }

  /**
   * Saves validation report to file
   * @param {object} report - Validation report to save
   * @param {string} filePath - Path to save report
   * @returns {boolean} Success status
   */
  saveValidationReport(report, filePath) {
    try {
      fs.writeFileSync(filePath, JSON.stringify(report, null, 2));
      return true;
    } catch (error) {
      console.error(`Failed to save validation report: ${error.message}`);
      return false;
    }
  }
}

// Example usage and test cases
function runROS2ValidationTests() {
  console.log('🧪 Running ROS 2 Code Example Validation Tests...\n');

  const validator = new ROS2ValidationTest();

  // Test cases for ROS 2 code examples
  const testCases = [
    {
      name: 'Basic ROS 2 Python Node',
      language: 'python',
      code: `
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
      `
    },
    {
      name: 'Simulation-specific Code',
      language: 'python',
      code: `
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity

class GazeboSpawner(Node):
    def __init__(self):
        super().__init__('gazebo_spawner')
        self.cli = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
      `
    },
    {
      name: 'Hardware-specific Code',
      language: 'python',
      code: `
import rclpy
from rclpy.node import Node
from hardware_interface.msg import HardwareCommand

class HardwareInterface(Node):
    def __init__(self):
        super().__init__('hardware_interface')
        self.sub = self.create_subscription(
            HardwareCommand,
            'hardware_commands',
            self.command_callback,
            10
        )
      `
    },
    {
      name: 'Basic ROS 2 C++ Node',
      language: 'cpp',
      code: `
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello World: " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
      `
    }
  ];

  // Run batch validation
  const results = validator.runBatchValidation(testCases);

  console.log('📊 Validation Results Summary:');
  console.log(`Total examples tested: ${results.length}`);
  console.log(`Passed: ${results.filter(r => r.validation.passed).length}`);
  console.log(`Failed: ${results.filter(r => !r.validation.passed).length}`);
  console.log('');

  // Show detailed results
  results.forEach((result, index) => {
    console.log(`Test ${index + 1}: ${result.name}`);
    console.log(`  ROS 2 Compatible: ${result.validation.ros2Compatible ? '✅' : '❌'}`);
    console.log(`  Simulation Compatible: ${result.validation.simulationCompatible ? '✅' : '❌'}`);
    console.log(`  Hardware Compatible: ${result.validation.hardwareCompatible ? '✅' : '❌'}`);
    console.log(`  Overall Score: ${result.validation.overallScore.toFixed(2)}/100`);
    console.log(`  Status: ${result.validation.passed ? '✅ PASS' : '❌ FAIL'}`);

    if (result.validation.errors.length > 0) {
      console.log(`  Errors: ${result.validation.errors.length}`);
      result.validation.errors.forEach(error => console.log(`    - ${error}`));
    }

    if (result.validation.warnings.length > 0) {
      console.log(`  Warnings: ${result.validation.warnings.length}`);
      result.validation.warnings.forEach(warning => console.log(`    - ${warning}`));
    }
    console.log('');
  });

  // Generate comprehensive report
  const report = validator.generateValidationReport(validator.validationResults);
  console.log('📈 Overall Statistics:');
  console.log(`Pass rate: ${report.passRate.toFixed(2)}%`);
  console.log(`Average score: ${report.averageScore.toFixed(2)}`);
  console.log(`ROS 2 compatible: ${report.compatibilityBreakdown.ros2Compatible}/${report.totalTests}`);
  console.log(`Simulation compatible: ${report.compatibilityBreakdown.simulationCompatible}/${report.totalTests}`);
  console.log(`Hardware compatible: ${report.compatibilityBreakdown.hardwareCompatible}/${report.totalTests}`);

  // Save report
  const reportPath = './ros2_validation_report.json';
  const saveSuccess = validator.saveValidationReport(report, reportPath);
  if (saveSuccess) {
    console.log(`\n📄 Validation report saved to: ${reportPath}`);
  }

  return report;
}

// Run tests if this file is executed directly
if (require.main === module) {
  runROS2ValidationTests();
}

module.exports = { ROS2ValidationTest, runROS2ValidationTests };