/**
 * Test suite for Jetson NX Code Example Validation Service
 * Tests validation of code examples for NVIDIA Jetson NX platform compatibility
 */

const JetsonValidationService = require('../services/jetson-validation-service');

function runTests() {
  console.log('🧪 Running Jetson NX Code Validation Tests...\n');

  let testCount = 0;
  let passedCount = 0;

  // Test 1: Valid Python code example
  testCount++;
  console.log(`Test ${testCount}: Valid Python code example`);
  const validPythonCode = `
import rclpy
from rclpy.node import Node
import cv2
import numpy as np

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.cap = cv2.VideoCapture(0)

    def process_frame(self):
        ret, frame = self.cap.read()
        if ret:
            # Small array operations suitable for Jetson
            processed = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            return processed
        return None
`;

  const result1 = JetsonValidationService.validateCodeExample(validPythonCode, 'python', 'ros_node');
  console.log(`  Result: ${result1.isValid ? '✅ PASS' : '❌ FAIL'}`);
  console.log(`  Errors: ${result1.errors.length}, Warnings: ${result1.warnings.length}`);
  if (!result1.isValid) {
    console.log(`  Errors: ${JSON.stringify(result1.errors)}`);
  }
  if (result1.isValid) passedCount++;
  console.log('');

  // Test 2: Python code with memory issue
  testCount++;
  console.log(`Test ${testCount}: Python code with potential memory issue`);
  const memoryIntensiveCode = `
import numpy as np

# This creates a very large array that might exceed Jetson limits
large_array = np.zeros((5000, 5000), dtype=np.float64)  # ~190MB
result = np.random.random((10000, 10000))  # ~760MB
`;

  const result2 = JetsonValidationService.validateCodeExample(memoryIntensiveCode, 'python');
  console.log(`  Result: ${result2.warnings.length > 0 ? '✅ PASS (detected warnings)' : '❌ FAIL (should detect warnings)'}`);
  console.log(`  Errors: ${result2.errors.length}, Warnings: ${result2.warnings.length}`);
  if (result2.warnings.length > 0) {
    console.log(`  Warnings: ${JSON.stringify(result2.warnings)}`);
    passedCount++;
  }
  console.log('');

  // Test 3: Invalid architecture-specific bash code
  testCount++;
  console.log(`Test ${testCount}: Bash code with invalid architecture`);
  const invalidBashCode = `
# This command is x86_64 specific and won't work on Jetson NX (ARM64)
if [ "$(uname -m)" = "x86_64" ]; then
    echo "Running on x86_64"
else
    echo "Running on ARM64"
fi
`;

  const result3 = JetsonValidationService.validateCodeExample(invalidBashCode, 'bash');
  console.log(`  Result: ${result3.errors.length > 0 ? '✅ PASS (detected errors)' : '❌ FAIL (should detect errors)'}`);
  console.log(`  Errors: ${result3.errors.length}, Warnings: ${result3.warnings.length}`);
  if (result3.errors.length > 0) {
    console.log(`  Errors: ${JSON.stringify(result3.errors)}`);
    passedCount++;
  }
  console.log('');

  // Test 4: Valid C++ code for Jetson
  testCount++;
  console.log(`Test ${testCount}: Valid C++ code for Jetson`);
  const validCppCode = `
#include <rclcpp/rclcpp.hpp>
#include <iostream>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("jetson_node");

    RCLCPP_INFO(node->get_logger(), "Hello from Jetson NX!");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
`;

  const result4 = JetsonValidationService.validateCodeExample(validCppCode, 'cpp', 'ros_node');
  console.log(`  Result: ${result4.isValid ? '✅ PASS' : '❌ FAIL'}`);
  console.log(`  Errors: ${result4.errors.length}, Warnings: ${result4.warnings.length}`);
  if (result4.isValid) passedCount++;
  console.log('');

  // Test 5: C++ code with large memory allocation
  testCount++;
  console.log(`Test ${testCount}: C++ code with large memory allocation`);
  const largeAllocationCode = `
#include <iostream>
#include <new>

int main() {
    try {
        // Attempt to allocate a very large array
        const size_t size = 2000000; // 2M elements
        int* large_array = new int[size];

        for(size_t i = 0; i < size; ++i) {
            large_array[i] = i;
        }

        delete[] large_array;
    } catch (const std::bad_alloc& e) {
        std::cerr << "Allocation failed: " << e.what() << std::endl;
    }

    return 0;
}
`;

  const result5 = JetsonValidationService.validateCodeExample(largeAllocationCode, 'cpp');
  console.log(`  Result: ${result5.warnings.length > 0 ? '✅ PASS (detected warnings)' : '❌ FAIL (should detect warnings)'}`);
  console.log(`  Errors: ${result5.errors.length}, Warnings: ${result5.warnings.length}`);
  if (result5.warnings.length > 0) {
    console.log(`  Warnings: ${JSON.stringify(result5.warnings)}`);
    passedCount++;
  }
  console.log('');

  // Test 6: Test file validation with a Python file
  testCount++;
  console.log(`Test ${testCount}: File validation test`);
  try {
    // Create a temporary Python file to test file validation
    const tempPythonFile = './temp_test.py';
    const fs = require('fs');
    fs.writeFileSync(tempPythonFile, '# Test Python file\nprint("Hello Jetson")\n');

    const fileResult = JetsonValidationService.validateCodeFile(tempPythonFile);
    console.log(`  Result: ${fileResult && fileResult.errors && fileResult.errors.length === 0 ? '✅ PASS' : '❌ FAIL'}`);
    console.log(`  Errors: ${fileResult && fileResult.errors ? fileResult.errors.length : 'undefined'}, Warnings: ${fileResult && fileResult.warnings ? fileResult.warnings.length : 'undefined'}`);
    if (fileResult && fileResult.errors && fileResult.errors.length === 0) passedCount++;

    // Clean up
    fs.unlinkSync(tempPythonFile);
  } catch (error) {
    console.log(`  Result: ❌ FAIL (error: ${error.message})`);
  }
  console.log('');

  // Test 7: Test unsupported language
  testCount++;
  console.log(`Test ${testCount}: Unsupported language test`);
  const result7 = JetsonValidationService.validateCodeExample('print("hello")', 'go');
  console.log(`  Result: ${result7.errors.length > 0 ? '✅ PASS (detected unsupported language)' : '❌ FAIL (should detect unsupported language)'}`);
  if (result7.errors.length > 0) passedCount++;
  console.log('');

  // Summary
  console.log('📊 Test Summary:');
  console.log(`Total tests: ${testCount}`);
  console.log(`Passed: ${passedCount}`);
  console.log(`Failed: ${testCount - passedCount}`);
  console.log(`Success rate: ${Math.round((passedCount / testCount) * 100)}%`);

  if (passedCount === testCount) {
    console.log('🎉 All tests passed!');
  } else {
    console.log('⚠️ Some tests failed.');
  }
}

// Run the tests if this file is executed directly
if (require.main === module) {
  runTests();
}

module.exports = { runTests };