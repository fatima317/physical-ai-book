/**
 * Jetson NX Code Execution Test Framework
 * Tests code examples for execution on Jetson NX platform
 * This framework validates code execution in simulated Jetson environment
 */

const { spawn, exec } = require('child_process');
const fs = require('fs');
const path = require('path');
const os = require('os');

class JetsonCodeExecutionTest {
  constructor() {
    this.jetsonSpecs = {
      architecture: 'aarch64', // ARM64
      cores: 6, // Carmel ARMv8.2 6-core CPU
      gpu: 'NVIDIA Jetson GPU',
      memory: '4GB/8GB', // Depending on model
      os: 'Linux (Ubuntu based)',
      cudaCores: '1024', // For Jetson NX
      tensorCores: '32'  // For Jetson NX
    };

    this.testResults = [];
  }

  /**
   * Test Python code execution on Jetson-like environment
   * @param {string} code - Python code to test
   * @param {object} requirements - Python requirements
   * @returns {object} Test result
   */
  async testPythonExecution(code, requirements = {}) {
    const result = {
      language: 'python',
      passed: false,
      errors: [],
      warnings: [],
      output: '',
      executionTime: 0
    };

    try {
      // Create a temporary Python file
      const tempFile = path.join(os.tmpdir(), `jetson_test_${Date.now()}.py`);
      fs.writeFileSync(tempFile, code);

      // Start timing
      const startTime = Date.now();

      // Execute the Python code
      const pythonProcess = spawn('python3', [tempFile]);

      let output = '';
      let errorOutput = '';

      pythonProcess.stdout.on('data', (data) => {
        output += data.toString();
      });

      pythonProcess.stderr.on('data', (data) => {
        errorOutput += data.toString();
      });

      const exitCode = await new Promise((resolve) => {
        pythonProcess.on('close', (code) => {
          resolve(code);
        });
      });

      result.executionTime = Date.now() - startTime;
      result.output = output;

      if (errorOutput) {
        result.errors.push(errorOutput);
      }

      // Check if execution was successful
      result.passed = exitCode === 0;

      // Clean up
      fs.unlinkSync(tempFile);

    } catch (error) {
      result.errors.push(`Execution error: ${error.message}`);
    }

    return result;
  }

  /**
   * Test if code uses Jetson-specific libraries correctly
   * @param {string} code - Code to analyze
   * @returns {array} Issues found
   */
  analyzeJetsonSpecificUsage(code) {
    const issues = [];

    // Check for common Jetson-specific libraries
    if (code.includes('jetson.')) {
      // Check if jetson namespace is properly imported
      if (!code.includes('import jetson') && !code.includes('from jetson')) {
        issues.push('Using jetson namespace without proper import');
      }
    }

    // Check for CUDA usage
    if (code.includes('cuda') && !code.includes('torch.cuda') && !code.includes('cupy')) {
      issues.push('Direct CUDA usage without proper framework (PyTorch/CuPy)');
    }

    // Check for hardware-specific operations
    if (code.includes('nvpmodel') || code.includes('jetson_clocks')) {
      issues.push('Using Jetson power management commands - ensure proper permissions');
    }

    return issues;
  }

  /**
   * Test code for ARM64 compatibility
   * @param {string} code - Code to analyze
   * @returns {array} Issues found
   */
  analyzeArm64Compatibility(code) {
    const issues = [];

    // Check for x86_64 specific code
    if (code.includes('__x86_64__') || code.includes('x86_64')) {
      issues.push('x86_64 specific code detected - not compatible with ARM64 Jetson');
    }

    // Check for architecture-specific libraries
    if (code.includes('intel.') || code.includes('mkl') || code.includes('AVX')) {
      issues.push('x86-specific optimizations detected - not compatible with ARM64 Jetson');
    }

    return issues;
  }

  /**
   * Test memory usage of code
   * @param {string} code - Code to analyze
   * @returns {array} Memory-related issues
   */
  analyzeMemoryUsage(code) {
    const issues = [];

    // Check for large array allocations
    const largeArrayPattern = /(?:np\.(?:zeros|ones|random\.[^(]*\()|malloc\(|new \w+\[)(\d{7,})/g;
    let match;
    while ((match = largeArrayPattern.exec(code)) !== null) {
      const size = parseInt(match[1]);
      if (size > 50000000) { // More than 50M elements
        issues.push(`Potentially large memory allocation (${size} elements) for Jetson memory constraints`);
      }
    }

    return issues;
  }

  /**
   * Comprehensive test of code execution on Jetson
   * @param {string} code - Code to test
   * @param {string} language - Programming language
   * @param {object} metadata - Additional metadata
   * @returns {object} Comprehensive test result
   */
  async runComprehensiveTest(code, language = 'python', metadata = {}) {
    const result = {
      code: code.substring(0, 100) + '...', // First 100 chars
      language,
      metadata,
      execution: null,
      jetsonSpecificIssues: [],
      arm64CompatibilityIssues: [],
      memoryIssues: [],
      overallScore: 0,
      passed: false
    };

    // Run execution test
    if (language.toLowerCase() === 'python') {
      result.execution = await this.testPythonExecution(code);
    }

    // Analyze Jetson-specific usage
    result.jetsonSpecificIssues = this.analyzeJetsonSpecificUsage(code);

    // Analyze ARM64 compatibility
    result.arm64CompatibilityIssues = this.analyzeArm64Compatibility(code);

    // Analyze memory usage
    result.memoryIssues = this.analyzeMemoryUsage(code);

    // Calculate overall score
    const totalIssues = result.jetsonSpecificIssues.length +
                       result.arm64CompatibilityIssues.length +
                       result.memoryIssues.length;

    // Base score on execution success and issue count
    let score = result.execution?.passed ? 70 : 0;
    score -= Math.min(totalIssues * 10, 70); // Deduct up to 70 points for issues
    result.overallScore = Math.max(0, score);

    result.passed = result.overallScore >= 60;

    this.testResults.push(result);
    return result;
  }

  /**
   * Run tests on a set of code examples
   * @param {array} codeExamples - Array of code examples to test
   * @returns {array} Test results
   */
  async runBatchTests(codeExamples) {
    const results = [];

    for (const example of codeExamples) {
      const result = await this.runComprehensiveTest(
        example.code,
        example.language || 'python',
        example.metadata || {}
      );
      results.push(result);
    }

    return results;
  }

  /**
   * Generate a test report
   * @param {array} results - Test results to include in report
   * @returns {object} Test report
   */
  generateReport(results = this.testResults) {
    const totalTests = results.length;
    const passedTests = results.filter(r => r.passed).length;
    const failedTests = totalTests - passedTests;

    const report = {
      timestamp: new Date().toISOString(),
      totalTests,
      passedTests,
      failedTests,
      passRate: totalTests > 0 ? (passedTests / totalTests) * 100 : 0,
      averageScore: results.reduce((sum, r) => sum + r.overallScore, 0) / totalTests || 0,
      summary: {
        jetsonSpecificIssues: results.reduce((sum, r) => sum + r.jetsonSpecificIssues.length, 0),
        arm64CompatibilityIssues: results.reduce((sum, r) => sum + r.arm64CompatibilityIssues.length, 0),
        memoryIssues: results.reduce((sum, r) => sum + r.memoryIssues.length, 0)
      },
      detailedResults: results
    };

    return report;
  }

  /**
   * Save test report to file
   * @param {object} report - Test report to save
   * @param {string} filePath - Path to save report
   * @returns {boolean} Success status
   */
  saveReport(report, filePath) {
    try {
      fs.writeFileSync(filePath, JSON.stringify(report, null, 2));
      return true;
    } catch (error) {
      console.error(`Failed to save report: ${error.message}`);
      return false;
    }
  }
}

// Example usage and test cases
async function runJetsonExecutionTests() {
  console.log('🧪 Running Jetson NX Code Execution Tests...\n');

  const tester = new JetsonCodeExecutionTest();

  // Test cases
  const testCases = [
    {
      name: 'Simple Python code',
      code: 'print("Hello Jetson NX!")',
      language: 'python',
      metadata: { category: 'basic', complexity: 'low' }
    },
    {
      name: 'NumPy array operations',
      code: `
import numpy as np
a = np.random.random((1000, 1000))
b = np.random.random((1000, 1000))
c = np.dot(a, b)
print(f"Matrix multiplication result shape: {c.shape}")
      `,
      language: 'python',
      metadata: { category: 'math', complexity: 'medium' }
    },
    {
      name: 'OpenCV operations',
      code: `
import cv2
import numpy as np

# Create a simple image
img = np.zeros((480, 640, 3), dtype=np.uint8)
cv2.rectangle(img, (100, 100), (200, 200), (0, 255, 0), 2)
print("OpenCV operations completed successfully")
      `,
      language: 'python',
      metadata: { category: 'vision', complexity: 'medium' }
    },
    {
      name: 'ARM64 incompatible code',
      code: `
#ifdef __x86_64__
  print("This is x86_64 specific code")
#else
  print("This is ARM64 compatible code")
#endif
      `,
      language: 'python', // This is actually C/C++ style but for testing
      metadata: { category: 'compatibility', complexity: 'low' }
    }
  ];

  // Run tests
  const results = await tester.runBatchTests(testCases);

  // Generate report
  const report = tester.generateReport(results);

  console.log('📊 Test Results Summary:');
  console.log(`Total tests: ${report.totalTests}`);
  console.log(`Passed: ${report.passedTests}`);
  console.log(`Failed: ${report.failedTests}`);
  console.log(`Pass rate: ${report.passRate.toFixed(2)}%`);
  console.log(`Average score: ${report.averageScore.toFixed(2)}`);

  // Show detailed results
  console.log('\n📝 Detailed Results:');
  results.forEach((result, index) => {
    console.log(`\nTest ${index + 1}: ${testCases[index].name}`);
    console.log(`  Status: ${result.passed ? '✅ PASS' : '❌ FAIL'}`);
    console.log(`  Score: ${result.overallScore}/100`);
    console.log(`  Execution: ${result.execution?.passed ? '✅' : '❌'}`);
    console.log(`  Jetson Issues: ${result.jetsonSpecificIssues.length}`);
    console.log(`  ARM64 Issues: ${result.arm64CompatibilityIssues.length}`);
    console.log(`  Memory Issues: ${result.memoryIssues.length}`);
  });

  // Save report
  const reportPath = './jetson_execution_test_report.json';
  const saveSuccess = tester.saveReport(report, reportPath);
  if (saveSuccess) {
    console.log(`\n📄 Report saved to: ${reportPath}`);
  }

  return report;
}

// Run tests if this file is executed directly
if (require.main === module) {
  runJetsonExecutionTests().catch(console.error);
}

module.exports = { JetsonCodeExecutionTest, runJetsonExecutionTests };