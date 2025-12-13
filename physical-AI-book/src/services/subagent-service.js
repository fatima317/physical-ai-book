/**
 * Subagent Execution Service
 * Manages the execution of specialized AI subagents for different robotics tasks
 */

const fs = require('fs');
const path = require('path');
const { spawn } = require('child_process');
const StorageService = require('./storage-service');

class SubagentService {
  constructor() {
    this.storageService = new StorageService();
    this.subagentsPath = path.join(process.cwd(), '.claude', 'commands');
    this.availableSubagents = new Map();
    this.executionHistory = new Map();
    this.maxExecutionTime = 30000; // 30 seconds max execution time
    this.maxConcurrentExecutions = 5;

    this.initializeSubagents();
  }

  /**
   * Initializes available subagents by scanning the commands directory
   */
  initializeSubagents() {
    try {
      if (fs.existsSync(this.subagentsPath)) {
        const files = fs.readdirSync(this.subagentsPath);
        for (const file of files) {
          if (file.endsWith('.claude')) {
            const subagentName = file.replace('.claude', '');
            const subagentPath = path.join(this.subagentsPath, file);
            const subagentContent = fs.readFileSync(subagentPath, 'utf8');

            // Extract subagent description from the file
            const descriptionMatch = subagentContent.match(/^# (.+)$/m);
            const description = descriptionMatch ? descriptionMatch[1] : 'No description available';

            this.availableSubagents.set(subagentName, {
              name: subagentName,
              path: subagentPath,
              description: description,
              content: subagentContent,
              lastModified: fs.statSync(subagentPath).mtime
            });
          }
        }
      }
    } catch (error) {
      console.error(`Error initializing subagents: ${error.message}`);
    }
  }

  /**
   * Gets available subagents
   * @returns {Array} List of available subagents
   */
  getAvailableSubagents() {
    return Array.from(this.availableSubagents.values()).map(subagent => ({
      name: subagent.name,
      description: subagent.description,
      lastModified: subagent.lastModified
    }));
  }

  /**
   * Executes a subagent with the given input
   * @param {string} subagentName - Name of the subagent to execute
   * @param {string} input - Input for the subagent
   * @param {object} options - Execution options
   * @returns {Promise<object>} Execution result
   */
  async executeSubagent(subagentName, input, options = {}) {
    return new Promise((resolve, reject) => {
      // Validate subagent exists
      if (!this.availableSubagents.has(subagentName)) {
        return reject(new Error(`Subagent '${subagentName}' not found`));
      }

      const subagent = this.availableSubagents.get(subagentName);
      const executionId = this.generateExecutionId();

      // Create execution record
      const executionRecord = {
        id: executionId,
        subagent: subagentName,
        input: input,
        timestamp: new Date().toISOString(),
        status: 'running',
        output: '',
        error: null,
        executionTime: 0
      };

      this.executionHistory.set(executionId, executionRecord);

      // Start timing
      const startTime = Date.now();

      // For now, we'll simulate the subagent execution by returning its prompt
      // In a real implementation, this would interface with an LLM API
      try {
        // Simulate processing time
        setTimeout(() => {
          const endTime = Date.now();
          const executionTime = endTime - startTime;

          // For simulation purposes, return a response based on the subagent's purpose
          let output = '';
          if (subagentName === 'ros_expert') {
            output = this.generateROS2Response(input);
          } else if (subagentName === 'gazebo_builder') {
            output = this.generateGazeboResponse(input);
          } else if (subagentName === 'hardware_guide') {
            output = this.generateHardwareResponse(input);
          } else if (subagentName === 'isaac_trainer') {
            output = this.generateIsaacResponse(input);
          } else if (subagentName === 'vla_planner') {
            output = this.generateVLAResponse(input);
          } else {
            // Default response for unknown subagents
            output = `Processed input with ${subagentName}: ${input}\n\n${subagent.content.substring(0, 500)}...`;
          }

          executionRecord.output = output;
          executionRecord.status = 'completed';
          executionRecord.executionTime = executionTime;

          this.executionHistory.set(executionId, executionRecord);

          // Save to storage
          this.storageService.saveData(`subagent-executions/${executionId}.json`, executionRecord);

          resolve({
            executionId,
            subagent: subagentName,
            input,
            output,
            executionTime,
            status: 'completed'
          });
        }, Math.min(1000, input.length / 10)); // Simulate processing time based on input length
      } catch (error) {
        const endTime = Date.now();
        executionRecord.status = 'error';
        executionRecord.error = error.message;
        executionRecord.executionTime = endTime - startTime;

        this.executionHistory.set(executionId, executionRecord);
        reject(error);
      }
    });
  }

  /**
   * Generates a ROS 2 specific response based on input
   * @param {string} input - Input to process
   * @returns {string} Generated response
   */
  generateROS2Response(input) {
    // This would typically interface with an LLM API in a real implementation
    const ros2Patterns = [
      'node',
      'publisher',
      'subscriber',
      'service',
      'client',
      'topic',
      'message',
      'launch',
      'parameter',
      'tf',
      'action'
    ];

    const hasROS2Pattern = ros2Patterns.some(pattern =>
      input.toLowerCase().includes(pattern)
    );

    if (hasROS2Pattern) {
      return `# ROS 2 Response to: ${input.substring(0, 100)}...

Here's a ROS 2 implementation based on your request:

\`\`\`python
import rclpy
from rclpy.node import Node

class GeneratedNode(Node):
    def __init__(self):
        super().__init__('generated_node')
        # Implementation based on your requirements
        pass

def main(args=None):
    rclpy.init(args=args)
    node = GeneratedNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
\`\`\`

This implementation follows ROS 2 Humble best practices for the Jetson platform.
`;
    }

    return `# ROS 2 Expert Response

Based on your input: "${input.substring(0, 100)}..."

I can help you with ROS 2 development for the Jetson platform. Please provide more specific details about the node, message type, or functionality you need assistance with.`;
  }

  /**
   * Generates a Gazebo specific response based on input
   * @param {string} input - Input to process
   * @returns {string} Generated response
   */
  generateGazeboResponse(input) {
    const gazeboPatterns = [
      'sdf',
      'urdf',
      'world',
      'model',
      'plugin',
      'sensor',
      'physics',
      'simulation'
    ];

    const hasGazeboPattern = gazeboPatterns.some(pattern =>
      input.toLowerCase().includes(pattern)
    );

    if (hasGazeboPattern) {
      return `# Gazebo Response to: ${input.substring(0, 100)}...

Here's a Gazebo simulation configuration based on your request:

\`\`\`xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="generated_world">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <physics name="ode" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>
    <!-- Generated based on your requirements -->
  </world>
</sdf>
\`\`\`

This configuration is optimized for Jetson hardware constraints.
`;
    }

    return `# Gazebo Builder Response

Based on your input: "${input.substring(0, 100)}..."

I can help you create Gazebo simulations for the Jetson platform. Please provide more specific details about the world, model, or sensor configuration you need assistance with.`;
  }

  /**
   * Generates a hardware specific response based on input
   * @param {string} input - Input to process
   * @returns {string} Generated response
   */
  generateHardwareResponse(input) {
    const hardwarePatterns = [
      'jetson',
      'setup',
      'gpio',
      'i2c',
      'spi',
      'pin',
      'power',
      'thermal',
      'camera',
      'sensor',
      'motor'
    ];

    const hasHardwarePattern = hardwarePatterns.some(pattern =>
      input.toLowerCase().includes(pattern)
    );

    if (hasHardwarePattern) {
      return `# Hardware Response to: ${input.substring(0, 100)}...

Based on your hardware requirements, here's the setup guidance:

1. **Power Requirements**: Verify 19V/4A power supply for full Jetson NX performance
2. **Thermal Management**: Ensure active cooling solution is properly installed
3. **Component Connections**: Verify pin connections match specifications
4. **Software Configuration**: Install appropriate JetPack version for your sensors

**Safety Check**: Always verify connections before powering on.
`;
    }

    return `# Hardware Guide Response

Based on your input: "${input.substring(0, 100)}..."

I can help you with Jetson hardware setup and configuration. Please provide more specific details about the component, connection, or setup issue you need assistance with.`;
  }

  /**
   * Generates an Isaac Sim specific response based on input
   * @param {string} input - Input to process
   * @returns {string} Generated response
   */
  generateIsaacResponse(input) {
    return `# Isaac Sim Response to: ${input.substring(0, 100)}...

Based on your perception or VSLAM requirements, here's the approach:

1. **Environment Setup**: Configure Isaac Sim with appropriate sensors
2. **Synthetic Data Generation**: Use domain randomization for robust training
3. **Perception Pipeline**: Implement computer vision algorithms optimized for Jetson
4. **Validation**: Test in simulation before hardware deployment

For more specific implementation details, please provide additional context about your use case.
`;
  }

  /**
   * Generates a VLA (Vision-Language-Action) specific response based on input
   * @param {string} input - Input to process
   * @returns {string} Generated response
   */
  generateVLAResponse(input) {
    return `# VLA Planner Response to: ${input.substring(0, 100)}...

Based on your LLM-to-action pipeline requirements, here's the approach:

1. **Perception**: Process visual input using vision models
2. **Language Understanding**: Interpret commands using LLMs (like Whisper for speech)
3. **Action Planning**: Generate executable actions for the robot
4. **Execution**: Map planned actions to robot control commands

This pipeline integrates vision, language, and action for intelligent robot behavior.
`;
  }

  /**
   * Gets execution history
   * @param {string} subagentName - Optional subagent name to filter
   * @param {number} limit - Optional limit on results
   * @returns {Array} Execution history
   */
  getExecutionHistory(subagentName = null, limit = 50) {
    let history = Array.from(this.executionHistory.values());

    if (subagentName) {
      history = history.filter(exec => exec.subagent === subagentName);
    }

    // Sort by timestamp (newest first)
    history.sort((a, b) => new Date(b.timestamp) - new Date(a.timestamp));

    return history.slice(0, limit);
  }

  /**
   * Gets execution result by ID
   * @param {string} executionId - Execution ID
   * @returns {object} Execution result
   */
  getExecutionResult(executionId) {
    return this.executionHistory.get(executionId) || null;
  }

  /**
   * Generates a unique execution ID
   * @returns {string} Execution ID
   */
  generateExecutionId() {
    return `exec_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  }

  /**
   * Gets statistics about subagent usage
   * @returns {object} Usage statistics
   */
  getUsageStats() {
    const stats = {
      totalExecutions: this.executionHistory.size,
      subagentUsage: {},
      recentExecutions: []
    };

    // Count subagent usage
    for (const [id, execution] of this.executionHistory) {
      if (!stats.subagentUsage[execution.subagent]) {
        stats.subagentUsage[execution.subagent] = 0;
      }
      stats.subagentUsage[execution.subagent]++;
    }

    // Get recent executions
    const sortedExecutions = Array.from(this.executionHistory.values())
      .sort((a, b) => new Date(b.timestamp) - new Date(a.timestamp))
      .slice(0, 10);

    stats.recentExecutions = sortedExecutions;

    return stats;
  }

  /**
   * Clears execution history older than specified days
   * @param {number} days - Number of days to keep
   */
  clearOldHistory(days = 30) {
    const cutoffDate = new Date();
    cutoffDate.setDate(cutoffDate.getDate() - days);

    for (const [id, execution] of this.executionHistory) {
      if (new Date(execution.timestamp) < cutoffDate) {
        this.executionHistory.delete(id);
      }
    }
  }
}

module.exports = new SubagentService();