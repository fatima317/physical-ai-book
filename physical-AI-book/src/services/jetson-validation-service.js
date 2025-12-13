/**
 * Jetson NX Code Example Validation Service
 * Validates code examples for compatibility with NVIDIA Jetson NX platform
 */

const fs = require('fs');
const path = require('path');
const { spawn } = require('child_process');

class JetsonValidationService {
  constructor() {
    this.supportedLanguages = ['python', 'cpp', 'bash'];
    this.jetsonRequirements = {
      pythonVersion: '>=3.8, <=3.10',
      rosVersion: 'humble',
      hardwareConstraints: {
        maxMemoryUsage: '6GB', // Leave some memory for system processes
        gpuRequirements: 'CUDA-compatible operations only',
        armArchitecture: true
      }
    };
  }

  /**
   * Validates a code example for Jetson NX compatibility
   * @param {string} code - The code example to validate
   * @param {string} language - The programming language
   * @param {string} context - The context where the code will be used
   * @returns {object} Validation result with errors and warnings
   */
  validateCodeExample(code, language, context = '') {
    if (!this.supportedLanguages.includes(language.toLowerCase())) {
      return {
        isValid: false,
        errors: [`Language '${language}' is not supported for Jetson NX validation`],
        warnings: [],
        suggestions: []
      };
    }

    const errors = [];
    const warnings = [];
    const suggestions = [];

    // Language-specific validation
    switch (language.toLowerCase()) {
      case 'python':
        this._validatePythonCode(code, errors, warnings, suggestions);
        break;
      case 'cpp':
        this._validateCppCode(code, errors, warnings, suggestions);
        break;
      case 'bash':
        this._validateBashCode(code, errors, warnings, suggestions);
        break;
    }

    // General validation checks
    this._validateGeneralConstraints(code, errors, warnings, suggestions);

    // Context-specific validation
    if (context) {
      this._validateContextSpecific(code, context, errors, warnings, suggestions);
    }

    return {
      isValid: errors.length === 0,
      errors,
      warnings,
      suggestions,
      summary: {
        errorCount: errors.length,
        warningCount: warnings.length,
        suggestionCount: suggestions.length
      }
    };
  }

  /**
   * Validates Python code for Jetson NX compatibility
   */
  _validatePythonCode(code, errors, warnings, suggestions) {
    // Check for incompatible Python versions or modules
    if (code.includes('import tensorflow') && !code.includes('import tensorflow as') &&
        !code.includes('from tensorflow')) {
      // Check if TensorFlow version is compatible
      if (code.includes('tensorflow==') || code.includes('tf.__version__')) {
        // Validate TensorFlow version compatibility
        const tfVersionMatch = code.match(/tensorflow==(\d+\.\d+\.\d+)/);
        if (tfVersionMatch) {
          const version = tfVersionMatch[1];
          if (this._isIncompatibleTensorFlowVersion(version)) {
            warnings.push(`TensorFlow version ${version} may not be fully optimized for Jetson NX. Consider using JetPack-compatible versions.`);
          }
        }
      }
    }

    // Check for memory-intensive operations
    if (code.includes('np.zeros') || code.includes('np.ones') || code.includes('np.random')) {
      const largeArrayPattern = /(?:np\.(?:zeros|ones|random\..*)\s*\(\s*\(\s*(\d+)[, ]\s*(\d+)|np\.(?:zeros|ones|random\..*)\s*\(\s*(\d+)[, ]\s*(\d+))/g;
      let match;
      while ((match = largeArrayPattern.exec(code)) !== null) {
        const dims = match[1] && match[2] ? [parseInt(match[1]), parseInt(match[2])] :
                    match[3] && match[4] ? [parseInt(match[3]), parseInt(match[4])] : [];
        if (dims.length >= 2) {
          const size = dims[0] * dims[1];
          if (size > 1000000) { // More than 1M elements
            warnings.push(`Large array allocation (${size} elements) may exceed Jetson NX memory limits. Consider using smaller batch sizes or memory mapping.`);
          }
        }
      }
    }

    // Check for ARM-specific issues
    if (code.includes('cv2') && (code.includes('.so') || code.includes('cdll') || code.includes('CDLL'))) {
      warnings.push('Native library loading may have compatibility issues on ARM64 architecture. Verify ARM64 compatibility of native libraries.');
    }

    // Check for CUDA usage
    if (code.includes('cuda') || code.includes('cupy') || code.includes('torch.cuda')) {
      suggestions.push('Ensure CUDA operations are properly managed for Jetson NX GPU capabilities.');
    }

    // Check for resource-intensive operations
    if (code.includes('while True') || code.includes('while(1)')) {
      warnings.push('Infinite loops can cause system resource exhaustion. Add proper exit conditions and sleep intervals.');
    }
  }

  /**
   * Validates C++ code for Jetson NX compatibility
   */
  _validateCppCode(code, errors, warnings, suggestions) {
    // Check for memory allocation patterns
    if (code.includes('new ') || code.includes('malloc') || code.includes('calloc')) {
      // Pattern for new[] allocations: new type[size] - captures the expression in brackets
      const newArrayPattern = /new\s+\w+(?:\s*<[^>]*>)?\s*\[\s*([^[\]]+)\s*\]/g;
      let match;
      while ((match = newArrayPattern.exec(code)) !== null) {
        const expression = match[1].trim();

        // Look for direct large numbers in the expression
        const directNumberMatch = expression.match(/\b(\d{7,})\b/); // Numbers with 7+ digits
        if (directNumberMatch) {
          const size = parseInt(directNumberMatch[1]);
          if (size > 1000000) {
            warnings.push(`Large memory allocation detected (${size} elements) which may exceed Jetson NX memory limits.`);
          }
        }

        // Look for variable definitions in the code that might contain large values
        const varName = expression.trim();
        if (isNaN(varName)) { // If it's a variable name (not a direct number), try to find its definition
          const varPattern = new RegExp(`(?:const\\s+)?(?:size_t|int|long)\\s+${varName}\\s*=\\s*([^;]+)`, 'g');
          let varMatch;
          while ((varMatch = varPattern.exec(code)) !== null) {
            const varValue = varMatch[1].trim();
            const numMatch = varValue.match(/\b(\d{7,})\b/);
            if (numMatch) {
              const size = parseInt(numMatch[1]);
              if (size > 1000000) {
                warnings.push(`Large memory allocation detected (${size} elements) which may exceed Jetson NX memory limits.`);
              }
            }
          }
        }
      }

      // Pattern for malloc/calloc allocations: malloc(size) - captures the expression in parentheses
      const mallocPattern = /(?:malloc|calloc)\s*\(\s*([^)]+)\s*\)/g;
      while ((match = mallocPattern.exec(code)) !== null) {
        const expression = match[1].trim();

        // Look for direct large numbers in the expression
        const directNumberMatch = expression.match(/\b(\d{7,})\b/); // Numbers with 7+ digits
        if (directNumberMatch) {
          const size = parseInt(directNumberMatch[1]);
          if (size > 1000000) {
            warnings.push(`Large memory allocation detected (${size} bytes) which may exceed Jetson NX memory limits.`);
          }
        }

        // Look for variable definitions that might contain large values
        const varName = expression.replace(/\s/g, '').replace(/[^a-zA-Z0-9_]/g, ''); // Simplified extraction
        if (isNaN(varName)) {
          const varPattern = new RegExp(`(?:const\\s+)?(?:size_t|int|long)\\s+${varName}\\s*=\\s*([^;]+)`, 'g');
          let varMatch;
          while ((varMatch = varPattern.exec(code)) !== null) {
            const varValue = varMatch[1].trim();
            const numMatch = varValue.match(/\b(\d{7,})\b/);
            if (numMatch) {
              const size = parseInt(numMatch[1]);
              if (size > 1000000) {
                warnings.push(`Large memory allocation detected (${size} bytes) which may exceed Jetson NX memory limits.`);
              }
            }
          }
        }
      }
    }

    // Check for potentially problematic patterns
    // Check for large static array declarations
    const staticArrayPattern = /\w+\s+\w+\s*\[\s*(\d{7,})\s*\]/g;
    let arrayMatch;
    while ((arrayMatch = staticArrayPattern.exec(code)) !== null) {
      const size = parseInt(arrayMatch[1]);
      if (size > 1000000) {
        warnings.push(`Large static array allocation detected (${size} elements) which may exceed Jetson NX memory limits.`);
      }
    }

    // Check for threading
    if (code.includes('#include <thread>') || code.includes('std::thread')) {
      suggestions.push('Consider thread count limitations on Jetson NX (6-core CPU). Optimize thread usage for ARM architecture.');
    }

    // Check for ARM-specific compilation
    if (code.includes('#ifdef __x86_64__') || code.includes('#ifdef __i386__')) {
      warnings.push('Architecture-specific code detected. Ensure ARM64 compatibility for Jetson NX.');
    }
  }

  /**
   * Validates Bash code for Jetson NX compatibility
   */
  _validateBashCode(code, errors, warnings, suggestions) {
    // Check for architecture-specific commands
    if (code.includes('x86_64') || code.includes('amd64')) {
      errors.push('x86_64/amd64 specific commands are not compatible with Jetson NX ARM64 architecture.');
    }

    // Check for hardware-specific operations
    if (code.includes('/dev/ttyUSB') || code.includes('/dev/ttyACM')) {
      suggestions.push('Verify that the specified serial devices are available on your Jetson NX hardware setup.');
    }

    // Check for resource constraints
    if (code.includes('ulimit') || code.includes('nice') || code.includes('ionice')) {
      suggestions.push('Consider Jetson NX resource constraints when setting process priorities.');
    }
  }

  /**
   * Validates general constraints applicable to all languages
   */
  _validateGeneralConstraints(code, errors, warnings, suggestions) {
    // Check for ROS dependencies
    if (code.includes('rclcpp') || code.includes('rclpy') || code.includes('ros2')) {
      // Verify ROS 2 Humble compatibility
      if (code.includes('rclcpp::init') || code.includes('rclpy.init')) {
        suggestions.push('Ensure ROS 2 Humble packages are properly installed and sourced on Jetson NX.');
      }
    }

    // Check for GPU/CUDA usage
    if (code.includes('CUDA') || code.includes('cuda') || code.includes('GPU')) {
      suggestions.push('Verify CUDA compatibility with Jetson NX GPU (NVIDIA Carmel ARM64 CPU + NVIDIA GPU).');
    }

    // Check for power management
    if (code.includes('nvpmodel') || code.includes('jetson_clocks')) {
      warnings.push('Power management commands should only be used with appropriate permissions and understanding of system impact.');
    }
  }

  /**
   * Validates context-specific requirements
   */
  _validateContextSpecific(code, context, errors, warnings, suggestions) {
    switch (context.toLowerCase()) {
      case 'ros_node':
        if (!code.includes('rclpy') && !code.includes('rclcpp')) {
          warnings.push('ROS node code should include proper ROS client library imports.');
        }
        break;
      case 'perception':
        if (code.includes('cv2') && !code.includes('cv2.cuda')) {
          suggestions.push('Consider using CUDA-accelerated OpenCV functions for better performance on Jetson NX.');
        }
        break;
      case 'control':
        if (code.includes('time.sleep') && !code.includes('rclpy') && !code.includes('rclcpp')) {
          warnings.push('For real-time control, consider using ROS time utilities instead of blocking sleep calls.');
        }
        break;
    }
  }

  /**
   * Checks if TensorFlow version is incompatible with Jetson
   */
  _isIncompatibleTensorFlowVersion(version) {
    // This is a simplified check - in reality, would need more detailed compatibility matrix
    const [major, minor] = version.split('.').map(Number);
    return major > 2 || (major === 2 && minor > 12); // Assuming TF 2.12+ might have issues
  }

  /**
   * Validates a complete code file for Jetson NX compatibility
   */
  validateCodeFile(filePath) {
    if (!fs.existsSync(filePath)) {
      return {
        isValid: false,
        errors: [`File does not exist: ${filePath}`],
        warnings: [],
        suggestions: []
      };
    }

    const code = fs.readFileSync(filePath, 'utf8');
    const extension = path.extname(filePath).toLowerCase();

    let language;
    switch (extension) {
      case '.py':
        language = 'python';
        break;
      case '.cpp':
      case '.cc':
      case '.cxx':
      case '.c++':
        language = 'cpp';
        break;
      case '.sh':
      case '.bash':
        language = 'bash';
        break;
      default:
        return {
          isValid: false,
          errors: [`Unsupported file extension: ${extension}`],
          warnings: [],
          suggestions: []
        };
    }

    return this.validateCodeExample(code, language, `file:${filePath}`);
  }

  /**
   * Runs a code example in a safe environment to test Jetson NX compatibility
   */
  async runValidationTest(code, language, timeout = 10000) {
    return new Promise((resolve) => {
      const result = {
        success: false,
        output: '',
        error: '',
        timeout: false
      };

      let command, args;
      switch (language.toLowerCase()) {
        case 'python':
          command = 'python3';
          args = ['-c', code];
          break;
        case 'bash':
          command = 'bash';
          args = ['-c', code];
          break;
        default:
          result.error = `Unsupported language for execution: ${language}`;
          resolve(result);
          return;
      }

      const process = spawn(command, args, {
        stdio: ['pipe', 'pipe', 'pipe'],
        env: { ...process.env }
      });

      let stdout = '';
      let stderr = '';

      process.stdout.on('data', (data) => {
        stdout += data.toString();
      });

      process.stderr.on('data', (data) => {
        stderr += data.toString();
      });

      process.on('close', (code) => {
        result.success = code === 0;
        result.output = stdout;
        result.error = stderr;
        resolve(result);
      });

      process.on('error', (error) => {
        result.error = error.message;
        resolve(result);
      });

      // Set timeout
      setTimeout(() => {
        if (!result.timeout) {
          result.timeout = true;
          process.kill();
          resolve(result);
        }
      }, timeout);
    });
  }
}

module.exports = new JetsonValidationService();