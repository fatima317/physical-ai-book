/**
 * Simulation-to-Hardware Concept Transfer Test
 * Tests the conceptual alignment between simulation and hardware implementations
 */

const fs = require('fs');
const path = require('path');
const SimulationTransferValidationService = require('../services/simulation-transfer-validation-service');

class SimToHardwareConceptTest {
  constructor() {
    this.testResults = {
      kinematicAlignment: { passed: false, score: 0, details: {} },
      sensorAlignment: { passed: false, score: 0, details: {} },
      controlAlignment: { passed: false, score: 0, details: {} },
      timingAlignment: { passed: false, score: 0, details: {} },
      overall: { passed: false, score: 0 }
    };
  }

  /**
   * Test kinematic concept alignment between simulation and hardware
   * @param {object} simKinematics - Simulation kinematic parameters
   * @param {object} hwKinematics - Hardware kinematic parameters
   * @returns {object} Test result
   */
  testKinematicAlignment(simKinematics, hwKinematics) {
    const result = {
      passed: false,
      score: 0,
      errors: [],
      warnings: [],
      details: {}
    };

    // Validate wheel parameters
    if (simKinematics.wheelRadius && hwKinematics.wheelRadius) {
      const radiusDiff = Math.abs(simKinematics.wheelRadius - hwKinematics.wheelRadius);
      const tolerance = 0.005; // 5mm tolerance

      if (radiusDiff > tolerance) {
        result.errors.push(`Wheel radius mismatch: sim=${simKinematics.wheelRadius}m, hw=${hwKinematics.wheelRadius}m, diff=${radiusDiff.toFixed(4)}m`);
      } else {
        result.score += 25; // 25% for radius alignment
      }
    }

    if (simKinematics.wheelSeparation && hwKinematics.wheelSeparation) {
      const separationDiff = Math.abs(simKinematics.wheelSeparation - hwKinematics.wheelSeparation);
      const tolerance = 0.01; // 1cm tolerance

      if (separationDiff > tolerance) {
        result.errors.push(`Wheel separation mismatch: sim=${simKinematics.wheelSeparation}m, hw=${hwKinematics.wheelSeparation}m, diff=${separationDiff.toFixed(4)}m`);
      } else {
        result.score += 25; // 25% for separation alignment
      }
    }

    // Validate max speeds
    if (simKinematics.maxLinearSpeed && hwKinematics.maxLinearSpeed) {
      const speedRatio = Math.abs(simKinematics.maxLinearSpeed - hwKinematics.maxLinearSpeed) / hwKinematics.maxLinearSpeed;
      if (speedRatio > 0.2) { // 20% tolerance
        result.warnings.push(`Max linear speed ratio mismatch: ${speedRatio.toFixed(2)}`);
      } else {
        result.score += 25; // 25% for speed alignment
      }
    }

    if (simKinematics.maxAngularSpeed && hwKinematics.maxAngularSpeed) {
      const angularRatio = Math.abs(simKinematics.maxAngularSpeed - hwKinematics.maxAngularSpeed) / hwKinematics.maxAngularSpeed;
      if (angularRatio > 0.2) { // 20% tolerance
        result.warnings.push(`Max angular speed ratio mismatch: ${angularRatio.toFixed(2)}`);
      } else {
        result.score += 25; // 25% for angular speed alignment
      }
    }

    result.passed = result.errors.length === 0 && result.score >= 75;
    result.details = {
      wheelRadiusMatch: simKinematics.wheelRadius && hwKinematics.wheelRadius &&
                       Math.abs(simKinematics.wheelRadius - hwKinematics.wheelRadius) <= 0.005,
      wheelSeparationMatch: simKinematics.wheelSeparation && hwKinematics.wheelSeparation &&
                           Math.abs(simKinematics.wheelSeparation - hwKinematics.wheelSeparation) <= 0.01,
      speedAlignment: (simKinematics.maxLinearSpeed && hwKinematics.maxLinearSpeed &&
                      Math.abs(simKinematics.maxLinearSpeed - hwKinematics.maxLinearSpeed) / hwKinematics.maxLinearSpeed <= 0.2) ||
                     (simKinematics.maxAngularSpeed && hwKinematics.maxAngularSpeed &&
                      Math.abs(simKinematics.maxAngularSpeed - hwKinematics.maxAngularSpeed) / hwKinematics.maxAngularSpeed <= 0.2)
    };

    this.testResults.kinematicAlignment = result;
    return result;
  }

  /**
   * Test sensor concept alignment between simulation and hardware
   * @param {object} simSensors - Simulation sensor parameters
   * @param {object} hwSensors - Hardware sensor parameters
   * @returns {object} Test result
   */
  testSensorAlignment(simSensors, hwSensors) {
    const result = {
      passed: false,
      score: 0,
      errors: [],
      warnings: [],
      details: {}
    };

    // Test camera alignment
    if (simSensors.camera && hwSensors.camera) {
      // Resolution check
      if (simSensors.camera.resolution && hwSensors.camera.resolution) {
        const resMatch = simSensors.camera.resolution.width === hwSensors.camera.resolution.width &&
                        simSensors.camera.resolution.height === hwSensors.camera.resolution.height;

        if (!resMatch) {
          result.warnings.push(`Camera resolution mismatch: sim=${simSensors.camera.resolution.width}x${simSensors.camera.resolution.height}, ` +
                              `hw=${hwSensors.camera.resolution.width}x${hwSensors.camera.resolution.height}`);
        } else {
          result.score += 20;
        }
      }

      // Frame rate check
      if (simSensors.camera.frameRate && hwSensors.camera.frameRate) {
        const rateDiff = Math.abs(simSensors.camera.frameRate - hwSensors.camera.frameRate) / hwSensors.camera.frameRate;
        if (rateDiff > 0.1) { // 10% tolerance
          result.warnings.push(`Camera frame rate mismatch: sim=${simSensors.camera.frameRate}Hz, hw=${hwSensors.camera.frameRate}Hz`);
        } else {
          result.score += 20;
        }
      }

      // Field of view check
      if (simSensors.camera.fov && hwSensors.camera.fov) {
        const fovDiff = Math.abs(simSensors.camera.fov - hwSensors.camera.fov) / hwSensors.camera.fov;
        if (fovDiff > 0.1) { // 10% tolerance
          result.warnings.push(`Camera FOV mismatch: sim=${simSensors.camera.fov}°, hw=${hwSensors.camera.fov}°`);
        } else {
          result.score += 15;
        }
      }
    }

    // Test LiDAR alignment
    if (simSensors.lidar && hwSensors.lidar) {
      // Range check
      if (simSensors.lidar.range && hwSensors.lidar.range) {
        const rangeDiff = Math.abs(simSensors.lidar.range.max - hwSensors.lidar.range.max) / hwSensors.lidar.range.max;
        if (rangeDiff > 0.1) { // 10% tolerance
          result.errors.push(`LiDAR range mismatch: sim=${simSensors.lidar.range.max}m, hw=${hwSensors.lidar.range.max}m`);
        } else {
          result.score += 20;
        }
      }

      // FOV check
      if (simSensors.lidar.fov && hwSensors.lidar.fov) {
        const fovDiff = Math.abs(simSensors.lidar.fov - hwSensors.lidar.fov) / hwSensors.lidar.fov;
        if (fovDiff > 0.1) { // 10% tolerance
          result.errors.push(`LiDAR FOV mismatch: sim=${simSensors.lidar.fov}°, hw=${hwSensors.lidar.fov}°`);
        } else {
          result.score += 15;
        }
      }
    }

    // Test IMU alignment
    if (simSensors.imu && hwSensors.imu) {
      // Update rate check
      if (simSensors.imu.updateRate && hwSensors.imu.updateRate) {
        const rateDiff = Math.abs(simSensors.imu.updateRate - hwSensors.imu.updateRate) / hwSensors.imu.updateRate;
        if (rateDiff > 0.2) { // 20% tolerance
          result.warnings.push(`IMU update rate mismatch: sim=${simSensors.imu.updateRate}Hz, hw=${hwSensors.imu.updateRate}Hz`);
        } else {
          result.score += 10;
        }
      }
    }

    result.passed = result.errors.length === 0 && result.score >= 70;
    result.details = {
      cameraAlignment: simSensors.camera && hwSensors.camera && result.score >= 20,
      lidarAlignment: simSensors.lidar && hwSensors.lidar &&
                     Math.abs(simSensors.lidar.range.max - hwSensors.lidar.range.max) / hwSensors.lidar.range.max <= 0.1,
      imuAlignment: simSensors.imu && hwSensors.imu
    };

    this.testResults.sensorAlignment = result;
    return result;
  }

  /**
   * Test control concept alignment between simulation and hardware
   * @param {object} simControl - Simulation control parameters
   * @param {object} hwControl - Hardware control parameters
   * @returns {object} Test result
   */
  testControlAlignment(simControl, hwControl) {
    const result = {
      passed: false,
      score: 0,
      errors: [],
      warnings: [],
      details: {}
    };

    // Test control loop rates
    if (simControl.loopRate && hwControl.loopRate) {
      const rateDiff = Math.abs(simControl.loopRate - hwControl.loopRate) / hwControl.loopRate;
      if (rateDiff > 0.3) { // 30% tolerance for control loop
        result.warnings.push(`Control loop rate mismatch: sim=${simControl.loopRate}Hz, hw=${hwControl.loopRate}Hz`);
      } else {
        result.score += 30;
      }
    }

    // Test control algorithm compatibility
    if (simControl.algorithm && hwControl.algorithm) {
      const algMatch = simControl.algorithm.toLowerCase() === hwControl.algorithm.toLowerCase();
      if (!algMatch) {
        result.warnings.push(`Control algorithm mismatch: sim=${simControl.algorithm}, hw=${hwControl.algorithm}`);
      } else {
        result.score += 25;
      }
    }

    // Test PID parameters (if using PID control)
    if (simControl.pid && hwControl.pid) {
      const kpDiff = Math.abs(simControl.pid.kp - hwControl.pid.kp) / Math.max(simControl.pid.kp, hwControl.pid.kp, 1);
      const kiDiff = Math.abs(simControl.pid.ki - hwControl.pid.ki) / Math.max(simControl.pid.ki, hwControl.pid.ki, 1);
      const kdDiff = Math.abs(simControl.pid.kd - hwControl.pid.kd) / Math.max(simControl.pid.kd, hwControl.pid.kd, 1);

      if (kpDiff > 0.5 || kiDiff > 0.5 || kdDiff > 0.5) { // 50% tolerance
        result.warnings.push(`PID parameter mismatch: kp=${kpDiff.toFixed(2)}, ki=${kiDiff.toFixed(2)}, kd=${kdDiff.toFixed(2)}`);
      } else {
        result.score += 30;
      }
    }

    // Test actuator limits
    if (simControl.maxEffort && hwControl.maxEffort) {
      const effortDiff = Math.abs(simControl.maxEffort - hwControl.maxEffort) / hwControl.maxEffort;
      if (effortDiff > 0.2) { // 20% tolerance
        result.errors.push(`Max effort mismatch: sim=${simControl.maxEffort}, hw=${hwControl.maxEffort}`);
      } else {
        result.score += 15;
      }
    }

    result.passed = result.errors.length === 0 && result.score >= 70;
    result.details = {
      loopRateMatch: simControl.loopRate && hwControl.loopRate &&
                    Math.abs(simControl.loopRate - hwControl.loopRate) / hwControl.loopRate <= 0.3,
      algorithmMatch: simControl.algorithm && hwControl.algorithm &&
                     simControl.algorithm.toLowerCase() === hwControl.algorithm.toLowerCase(),
      pidMatch: simControl.pid && hwControl.pid &&
               Math.abs(simControl.pid.kp - hwControl.pid.kp) / Math.max(simControl.pid.kp, hwControl.pid.kp, 1) <= 0.5
    };

    this.testResults.controlAlignment = result;
    return result;
  }

  /**
   * Test timing concept alignment between simulation and hardware
   * @param {object} simTiming - Simulation timing parameters
   * @param {object} hwTiming - Hardware timing parameters
   * @returns {object} Test result
   */
  testTimingAlignment(simTiming, hwTiming) {
    const result = {
      passed: false,
      score: 0,
      errors: [],
      warnings: [],
      details: {}
    };

    // Test computation time alignment
    if (simTiming.computationTime && hwTiming.computationTime) {
      // Hardware should be slower than simulation (real-time constraints)
      if (hwTiming.computationTime < simTiming.computationTime) {
        result.warnings.push(`Hardware computation time (${hwTiming.computationTime}ms) is faster than simulation (${simTiming.computationTime}ms) - unexpected`);
      } else if (hwTiming.computationTime > simTiming.computationTime * 3) { // 3x tolerance
        result.warnings.push(`Hardware computation time (${hwTiming.computationTime}ms) is significantly slower than simulation (${simTiming.computationTime}ms)`);
      } else {
        result.score += 35;
      }
    }

    // Test latency alignment
    if (simTiming.latency && hwTiming.latency) {
      if (hwTiming.latency < simTiming.latency) {
        result.warnings.push(`Hardware latency (${hwTiming.latency}ms) is lower than simulation (${simTiming.latency}ms) - unexpected`);
      } else {
        result.score += 30;
      }
    }

    // Test real-time factor consideration
    if (simTiming.realTimeFactor && hwTiming.realTimeFactor) {
      // In simulation, real-time factor can be > 1, but on hardware it should be ~1
      if (simTiming.realTimeFactor <= 1 && hwTiming.realTimeFactor !== 1) {
        result.warnings.push(`Real-time factor mismatch: sim=${simTiming.realTimeFactor}, hw should be ~1`);
      } else {
        result.score += 20;
      }
    }

    // Test sensor synchronization
    if (simTiming.sensorSync && hwTiming.sensorSync) {
      if (simTiming.sensorSync !== hwTiming.sensorSync) {
        result.warnings.push(`Sensor synchronization approach differs: sim=${simTiming.sensorSync}, hw=${hwTiming.sensorSync}`);
      } else {
        result.score += 15;
      }
    }

    result.passed = result.errors.length === 0 && result.score >= 70;
    result.details = {
      computationTimeAligned: simTiming.computationTime && hwTiming.computationTime &&
                            hwTiming.computationTime >= simTiming.computationTime,
      latencyConsidered: simTiming.latency && hwTiming.latency,
      realTimeFactorCorrect: simTiming.realTimeFactor && hwTiming.realTimeFactor
    };

    this.testResults.timingAlignment = result;
    return result;
  }

  /**
   * Run comprehensive simulation-to-hardware concept transfer test
   * @param {object} simData - Complete simulation data
   * @param {object} hwData - Complete hardware data
   * @returns {object} Comprehensive test result
   */
  runComprehensiveTest(simData, hwData) {
    // Run individual tests
    const kinematicResult = this.testKinematicAlignment(
      simData.kinematics || {},
      hwData.kinematics || {}
    );

    const sensorResult = this.testSensorAlignment(
      simData.sensors || {},
      hwData.sensors || {}
    );

    const controlResult = this.testControlAlignment(
      simData.control || {},
      hwData.control || {}
    );

    const timingResult = this.testTimingAlignment(
      simData.timing || {},
      hwData.timing || {}
    );

    // Calculate overall score
    const overallScore = (
      kinematicResult.score * 0.25 +
      sensorResult.score * 0.30 +
      controlResult.score * 0.25 +
      timingResult.score * 0.20
    );

    const overallPassed =
      kinematicResult.passed &&
      sensorResult.passed &&
      controlResult.passed &&
      timingResult.passed;

    this.testResults.overall = {
      passed: overallPassed,
      score: overallScore,
      details: {
        kinematic: kinematicResult,
        sensor: sensorResult,
        control: controlResult,
        timing: timingResult
      }
    };

    // Generate recommendations
    const recommendations = this.generateRecommendations(
      kinematicResult,
      sensorResult,
      controlResult,
      timingResult
    );

    return {
      passed: overallPassed,
      score: overallScore,
      results: this.testResults,
      recommendations,
      summary: {
        kinematic: kinematicResult.passed ? '✅ Aligned' : '❌ Not Aligned',
        sensor: sensorResult.passed ? '✅ Aligned' : '❌ Not Aligned',
        control: controlResult.passed ? '✅ Aligned' : '❌ Not Aligned',
        timing: timingResult.passed ? '✅ Aligned' : '❌ Not Aligned'
      }
    };
  }

  /**
   * Generate recommendations based on test results
   * @param {object} kinematicResult - Kinematic test result
   * @param {object} sensorResult - Sensor test result
   * @param {object} controlResult - Control test result
   * @param {object} timingResult - Timing test result
   * @returns {array} Recommendations
   */
  generateRecommendations(kinematicResult, sensorResult, controlResult, timingResult) {
    const recommendations = [];

    if (!kinematicResult.passed) {
      recommendations.push("Verify physical parameters (wheel radius, separation) match hardware specifications");
      recommendations.push("Ensure kinematic constraints are properly modeled in simulation");
    }

    if (!sensorResult.passed) {
      recommendations.push("Validate sensor specifications match between simulation and hardware");
      recommendations.push("Consider sensor noise and latency in simulation models");
    }

    if (!controlResult.passed) {
      recommendations.push("Review control algorithm compatibility between simulation and hardware");
      recommendations.push("Adjust control parameters for hardware constraints and delays");
    }

    if (!timingResult.passed) {
      recommendations.push("Account for computational delays and real-time constraints in simulation");
      recommendations.push("Validate timing assumptions with hardware performance benchmarks");
    }

    if (recommendations.length === 0) {
      recommendations.push("Simulation and hardware concepts are well aligned. Ready for transfer validation.");
    }

    return recommendations;
  }

  /**
   * Generate a detailed test report
   * @param {object} testResult - Result from runComprehensiveTest
   * @param {string} outputPath - Path to save report
   * @returns {boolean} Success status
   */
  generateTestReport(testResult, outputPath) {
    const report = {
      timestamp: new Date().toISOString(),
      testName: "Simulation-to-Hardware Concept Transfer Test",
      overallResult: {
        passed: testResult.passed,
        score: testResult.score,
        summary: testResult.summary
      },
      detailedResults: testResult.results,
      recommendations: testResult.recommendations,
      validationScore: SimulationTransferValidationService.calculateTransferScore(
        testResult.results.kinematicAlignment,
        testResult.results.sensorAlignment,
        testResult.results.timingAlignment
      )
    };

    try {
      fs.writeFileSync(outputPath, JSON.stringify(report, null, 2));
      return true;
    } catch (error) {
      console.error(`Failed to write test report: ${error.message}`);
      return false;
    }
  }
}

// Example usage and test cases
function runSimToHardwareConceptTests() {
  console.log('🧪 Running Simulation-to-Hardware Concept Transfer Tests...\n');

  const tester = new SimToHardwareConceptTest();

  // Example simulation and hardware data
  const exampleSimData = {
    kinematics: {
      wheelRadius: 0.033,
      wheelSeparation: 0.16,
      maxLinearSpeed: 0.5,
      maxAngularSpeed: 1.0
    },
    sensors: {
      camera: {
        resolution: { width: 640, height: 480 },
        frameRate: 30,
        fov: 60
      },
      lidar: {
        range: { max: 5.0 },
        fov: 270,
        resolution: 720
      },
      imu: {
        updateRate: 100
      }
    },
    control: {
      loopRate: 50,
      algorithm: 'PID',
      pid: { kp: 1.0, ki: 0.1, kd: 0.05 },
      maxEffort: 100
    },
    timing: {
      computationTime: 5,
      latency: 2,
      realTimeFactor: 1.0,
      sensorSync: true
    }
  };

  const exampleHwData = {
    kinematics: {
      wheelRadius: 0.033,  // Same as sim
      wheelSeparation: 0.16,  // Same as sim
      maxLinearSpeed: 0.45,  // Slightly lower for safety
      maxAngularSpeed: 0.9   // Slightly lower for safety
    },
    sensors: {
      camera: {
        resolution: { width: 640, height: 480 },  // Same as sim
        frameRate: 30,  // Same as sim
        fov: 60  // Same as sim
      },
      lidar: {
        range: { max: 4.8 },  // Slightly lower than sim
        fov: 270,  // Same as sim
        resolution: 720  // Same as sim
      },
      imu: {
        updateRate: 100  // Same as sim
      }
    },
    control: {
      loopRate: 40,  // Slightly lower for hardware constraints
      algorithm: 'PID',  // Same as sim
      pid: { kp: 0.9, ki: 0.12, kd: 0.04 },  // Slightly adjusted
      maxEffort: 95  // Slightly lower for safety
    },
    timing: {
      computationTime: 8,  // Higher on hardware
      latency: 5,  // Higher on hardware
      realTimeFactor: 1.0,  // Real-time on hardware
      sensorSync: true  // Same approach
    }
  };

  // Run comprehensive test
  const testResult = tester.runComprehensiveTest(exampleSimData, exampleHwData);

  console.log('📊 Test Results Summary:');
  console.log(`Overall Score: ${testResult.score.toFixed(2)}/100`);
  console.log(`Overall Status: ${testResult.passed ? '✅ PASSED' : '❌ FAILED'}`);
  console.log('');
  console.log('Component Status:');
  console.log(`  Kinematic: ${testResult.summary.kinematic}`);
  console.log(`  Sensor: ${testResult.summary.sensor}`);
  console.log(`  Control: ${testResult.summary.control}`);
  console.log(`  Timing: ${testResult.summary.timing}`);
  console.log('');

  console.log('💡 Recommendations:');
  testResult.recommendations.forEach((rec, index) => {
    console.log(`  ${index + 1}. ${rec}`);
  });

  // Generate and save report
  const reportPath = './sim_to_hardware_concept_test_report.json';
  const reportSaved = tester.generateTestReport(testResult, reportPath);
  if (reportSaved) {
    console.log(`\n📄 Report saved to: ${reportPath}`);
  }

  return testResult;
}

// Run tests if this file is executed directly
if (require.main === module) {
  runSimToHardwareConceptTests();
}

module.exports = { SimToHardwareConceptTest, runSimToHardwareConceptTests };