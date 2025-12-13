/**
 * Simulation-to-Hardware Transfer Validation Service
 * Validates that simulation behaviors match hardware behaviors for successful transfer
 */

const fs = require('fs');
const path = require('path');

class SimulationTransferValidationService {
  constructor() {
    this.validationMetrics = {
      kinematicAccuracy: 0.05, // 5% tolerance for kinematic validation
      sensorAccuracy: 0.1,     // 10% tolerance for sensor validation
      timingAccuracy: 0.2,     // 20% tolerance for timing validation
      controlAccuracy: 0.05    // 5% tolerance for control validation
    };

    this.hardwareProfiles = {
      'jetbot': {
        wheelRadius: 0.033,      // meters
        wheelSeparation: 0.16,   // meters
        maxLinearSpeed: 0.5,     // m/s
        maxAngularSpeed: 1.0,    // rad/s
        accelerationLimit: 2.0,  // m/s^2
        mass: 1.0,               // kg
        frictionCoefficient: 0.7 // unitless
      },
      'turtlebot3': {
        wheelRadius: 0.033,      // meters
        wheelSeparation: 0.287,  // meters
        maxLinearSpeed: 0.26,    // m/s
        maxAngularSpeed: 1.82,   // rad/s
        accelerationLimit: 1.0,  // m/s^2
        mass: 1.08,              // kg
        frictionCoefficient: 0.6 // unitless
      }
    };
  }

  /**
   * Validates kinematic behavior transfer from simulation to hardware
   * @param {object} simData - Simulation kinematic data
   * @param {object} hwData - Hardware kinematic data
   * @param {string} robotType - Type of robot (e.g., 'jetbot', 'turtlebot3')
   * @returns {object} Validation result
   */
  validateKinematicTransfer(simData, hwData, robotType = 'jetbot') {
    const errors = [];
    const warnings = [];
    const suggestions = [];

    if (!this.hardwareProfiles[robotType]) {
      errors.push(`Unknown robot type: ${robotType}`);
      return { isValid: false, errors, warnings, suggestions };
    }

    const profile = this.hardwareProfiles[robotType];
    const tolerance = this.validationMetrics.kinematicAccuracy;

    // Validate wheel parameters
    if (simData.wheelRadius && Math.abs(simData.wheelRadius - profile.wheelRadius) > tolerance * profile.wheelRadius) {
      errors.push(`Wheel radius mismatch: sim=${simData.wheelRadius}, expected=${profile.wheelRadius}±${tolerance * profile.wheelRadius}`);
    }

    if (simData.wheelSeparation && Math.abs(simData.wheelSeparation - profile.wheelSeparation) > tolerance * profile.wheelSeparation) {
      errors.push(`Wheel separation mismatch: sim=${simData.wheelSeparation}, expected=${profile.wheelSeparation}±${tolerance * profile.wheelSeparation}`);
    }

    // Validate kinematic responses
    if (simData.linearVelocity && hwData.linearVelocity) {
      const velocityError = Math.abs(simData.linearVelocity - hwData.linearVelocity) / profile.maxLinearSpeed;
      if (velocityError > tolerance) {
        errors.push(`Linear velocity mismatch: sim=${simData.linearVelocity}, hw=${hwData.linearVelocity}, error=${(velocityError * 100).toFixed(2)}%`);
      }
    }

    if (simData.angularVelocity && hwData.angularVelocity) {
      const angularError = Math.abs(simData.angularVelocity - hwData.angularVelocity) / profile.maxAngularSpeed;
      if (angularError > tolerance) {
        errors.push(`Angular velocity mismatch: sim=${simData.angularVelocity}, hw=${hwData.angularVelocity}, error=${(angularError * 100).toFixed(2)}%`);
      }
    }

    // Validate trajectory accuracy
    let avgPositionError = 0;
    if (simData.trajectory && hwData.trajectory) {
      const simPositions = simData.trajectory.positions || [];
      const hwPositions = hwData.trajectory.positions || [];

      if (simPositions.length !== hwPositions.length) {
        warnings.push(`Trajectory length mismatch: sim=${simPositions.length}, hw=${hwPositions.length}`);
      } else {
        let positionErrorSum = 0;
        for (let i = 0; i < Math.min(simPositions.length, hwPositions.length); i++) {
          const dx = simPositions[i].x - hwPositions[i].x;
          const dy = simPositions[i].y - hwPositions[i].y;
          const distance = Math.sqrt(dx * dx + dy * dy);
          positionErrorSum += distance;
        }

        avgPositionError = positionErrorSum / simPositions.length;
        if (avgPositionError > 0.1) { // More than 10cm average error
          warnings.push(`Trajectory deviation: avg error = ${avgPositionError.toFixed(3)}m`);
          suggestions.push(`Consider adjusting simulation parameters or implementing feedback control for trajectory correction.`);
        }
      }
    }

    return {
      isValid: errors.length === 0,
      errors,
      warnings,
      suggestions,
      metrics: {
        positionAccuracy: avgPositionError > 0 ? 1 - avgPositionError : 1,
        velocityAccuracy: simData.linearVelocity && hwData.linearVelocity ?
          1 - Math.abs(simData.linearVelocity - hwData.linearVelocity) / profile.maxLinearSpeed : 1
      }
    };
  }

  /**
   * Validates sensor data transfer from simulation to hardware
   * @param {object} simSensorData - Simulation sensor data
   * @param {object} hwSensorData - Hardware sensor data
   * @returns {object} Validation result
   */
  validateSensorTransfer(simSensorData, hwSensorData) {
    const errors = [];
    const warnings = [];
    const suggestions = [];

    const tolerance = this.validationMetrics.sensorAccuracy;

    // Validate camera data
    if (simSensorData.camera && hwSensorData.camera) {
      // Validate resolution
      if (simSensorData.camera.resolution && hwSensorData.camera.resolution) {
        if (simSensorData.camera.resolution.width !== hwSensorData.camera.resolution.width ||
            simSensorData.camera.resolution.height !== hwSensorData.camera.resolution.height) {
          warnings.push(`Camera resolution mismatch: sim=${simSensorData.camera.resolution.width}x${simSensorData.camera.resolution.height}, ` +
                       `hw=${hwSensorData.camera.resolution.width}x${hwSensorData.camera.resolution.height}`);
        }
      }

      // Validate frame rate
      if (simSensorData.camera.frameRate && hwSensorData.camera.frameRate) {
        const frameRateError = Math.abs(simSensorData.camera.frameRate - hwSensorData.camera.frameRate) / hwSensorData.camera.frameRate;
        if (frameRateError > tolerance) {
          warnings.push(`Camera frame rate mismatch: sim=${simSensorData.camera.frameRate}, hw=${hwSensorData.camera.frameRate}, ` +
                       `error=${(frameRateError * 100).toFixed(2)}%`);
        }
      }

      // Validate field of view
      if (simSensorData.camera.fov && hwSensorData.camera.fov) {
        const fovError = Math.abs(simSensorData.camera.fov - hwSensorData.camera.fov) / hwSensorData.camera.fov;
        if (fovError > tolerance) {
          warnings.push(`Camera FOV mismatch: sim=${simSensorData.camera.fov}, hw=${hwSensorData.camera.fov}, ` +
                       `error=${(fovError * 100).toFixed(2)}%`);
        }
      }
    }

    // Validate LiDAR data
    if (simSensorData.lidar && hwSensorData.lidar) {
      // Validate range
      if (simSensorData.lidar.range && hwSensorData.lidar.range) {
        const rangeError = Math.abs(simSensorData.lidar.range.max - hwSensorData.lidar.range.max) / hwSensorData.lidar.range.max;
        if (rangeError > tolerance) {
          errors.push(`LiDAR range mismatch: sim=${simSensorData.lidar.range.max}, hw=${hwSensorData.lidar.range.max}, ` +
                     `error=${(rangeError * 100).toFixed(2)}%`);
        }
      }

      // Validate field of view
      if (simSensorData.lidar.fov && hwSensorData.lidar.fov) {
        const fovError = Math.abs(simSensorData.lidar.fov - hwSensorData.lidar.fov) / hwSensorData.lidar.fov;
        if (fovError > tolerance) {
          errors.push(`LiDAR FOV mismatch: sim=${simSensorData.lidar.fov}, hw=${hwSensorData.lidar.fov}, ` +
                     `error=${(fovError * 100).toFixed(2)}%`);
        }
      }

      // Validate resolution
      if (simSensorData.lidar.resolution && hwSensorData.lidar.resolution) {
        const resolutionError = Math.abs(simSensorData.lidar.resolution - hwSensorData.lidar.resolution) / hwSensorData.lidar.resolution;
        if (resolutionError > tolerance) {
          warnings.push(`LiDAR resolution mismatch: sim=${simSensorData.lidar.resolution}, hw=${hwSensorData.lidar.resolution}, ` +
                       `error=${(resolutionError * 100).toFixed(2)}%`);
        }
      }
    }

    // Validate IMU data
    if (simSensorData.imu && hwSensorData.imu) {
      // Validate update rate
      if (simSensorData.imu.updateRate && hwSensorData.imu.updateRate) {
        const rateError = Math.abs(simSensorData.imu.updateRate - hwSensorData.imu.updateRate) / hwSensorData.imu.updateRate;
        if (rateError > tolerance) {
          warnings.push(`IMU update rate mismatch: sim=${simSensorData.imu.updateRate}, hw=${hwSensorData.imu.updateRate}, ` +
                       `error=${(rateError * 100).toFixed(2)}%`);
        }
      }

      // Validate noise characteristics
      if (simSensorData.imu.noise && hwSensorData.imu.noise) {
        const accelNoiseError = Math.abs(simSensorData.imu.noise.accel - hwSensorData.imu.noise.accel) / hwSensorData.imu.noise.accel;
        const gyroNoiseError = Math.abs(simSensorData.imu.noise.gyro - hwSensorData.imu.noise.gyro) / hwSensorData.imu.noise.gyro;

        if (accelNoiseError > tolerance * 2) {
          warnings.push(`IMU accelerometer noise mismatch: sim=${simSensorData.imu.noise.accel}, hw=${hwSensorData.imu.noise.accel}, ` +
                       `error=${(accelNoiseError * 100).toFixed(2)}%`);
        }

        if (gyroNoiseError > tolerance * 2) {
          warnings.push(`IMU gyroscope noise mismatch: sim=${simSensorData.imu.noise.gyro}, hw=${hwSensorData.imu.noise.gyro}, ` +
                       `error=${(gyroNoiseError * 100).toFixed(2)}%`);
        }
      }
    }

    return {
      isValid: errors.length === 0,
      errors,
      warnings,
      suggestions,
      metrics: {
        sensorAccuracy: 1 - Math.max(
          simSensorData.camera && hwSensorData.camera ?
            Math.abs(simSensorData.camera.frameRate - hwSensorData.camera.frameRate) / hwSensorData.camera.frameRate : 0,
          simSensorData.lidar && hwSensorData.lidar ?
            Math.abs(simSensorData.lidar.range.max - hwSensorData.lidar.range.max) / hwSensorData.lidar.range.max : 0
        )
      }
    };
  }

  /**
   * Validates timing and performance transfer
   * @param {object} simTiming - Simulation timing data
   * @param {object} hwTiming - Hardware timing data
   * @returns {object} Validation result
   */
  validateTimingTransfer(simTiming, hwTiming) {
    const errors = [];
    const warnings = [];
    const suggestions = [];

    const tolerance = this.validationMetrics.timingAccuracy;

    // Validate control loop timing
    if (simTiming.controlLoopRate && hwTiming.controlLoopRate) {
      const loopRateError = Math.abs(simTiming.controlLoopRate - hwTiming.controlLoopRate) / hwTiming.controlLoopRate;
      if (loopRateError > tolerance) {
        errors.push(`Control loop rate mismatch: sim=${simTiming.controlLoopRate}Hz, hw=${hwTiming.controlLoopRate}Hz, ` +
                   `error=${(loopRateError * 100).toFixed(2)}%`);
      }
    }

    // Validate computation time
    if (simTiming.computationTime && hwTiming.computationTime) {
      const compTimeError = Math.abs(simTiming.computationTime - hwTiming.computationTime) / hwTiming.computationTime;
      if (compTimeError > tolerance * 2) { // Higher tolerance for computation time
        warnings.push(`Computation time mismatch: sim=${simTiming.computationTime}ms, hw=${hwTiming.computationTime}ms, ` +
                     `error=${(compTimeError * 100).toFixed(2)}%`);
        suggestions.push('Consider optimizing algorithms for hardware constraints or adjusting simulation complexity.');
      }
    }

    // Validate latency
    if (simTiming.latency && hwTiming.latency) {
      const latencyError = Math.abs(simTiming.latency - hwTiming.latency) / hwTiming.latency;
      if (latencyError > tolerance * 3) { // Higher tolerance for latency
        warnings.push(`Latency mismatch: sim=${simTiming.latency}ms, hw=${hwTiming.latency}ms, ` +
                     `error=${(latencyError * 100).toFixed(2)}%`);
        suggestions.push('Account for hardware communication and processing latency in control algorithms.');
      }
    }

    return {
      isValid: errors.length === 0,
      errors,
      warnings,
      suggestions,
      metrics: {
        timingAccuracy: 1 - Math.max(
          simTiming.controlLoopRate && hwTiming.controlLoopRate ?
            Math.abs(simTiming.controlLoopRate - hwTiming.controlLoopRate) / hwTiming.controlLoopRate : 0,
          simTiming.computationTime && hwTiming.computationTime ?
            Math.abs(simTiming.computationTime - hwTiming.computationTime) / hwTiming.computationTime : 0
        )
      }
    };
  }

  /**
   * Performs comprehensive simulation-to-hardware validation
   * @param {object} simData - Complete simulation data
   * @param {object} hwData - Complete hardware data
   * @param {string} robotType - Type of robot
   * @returns {object} Comprehensive validation result
   */
  validateCompleteTransfer(simData, hwData, robotType = 'jetbot') {
    // Validate kinematics
    const kinematicResult = this.validateKinematicTransfer(
      simData.kinematics || {},
      hwData.kinematics || {},
      robotType
    );

    // Validate sensors
    const sensorResult = this.validateSensorTransfer(
      simData.sensors || {},
      hwData.sensors || {}
    );

    // Validate timing
    const timingResult = this.validateTimingTransfer(
      simData.timing || {},
      hwData.timing || {}
    );

    // Combine all results
    const allErrors = [...kinematicResult.errors, ...sensorResult.errors, ...timingResult.errors];
    const allWarnings = [...kinematicResult.warnings, ...sensorResult.warnings, ...timingResult.warnings];
    const allSuggestions = [...kinematicResult.suggestions, ...sensorResult.suggestions, ...timingResult.suggestions];

    const overallScore = this.calculateTransferScore(kinematicResult, sensorResult, timingResult);

    return {
      isValid: allErrors.length === 0,
      errors: allErrors,
      warnings: allWarnings,
      suggestions: allSuggestions,
      metrics: {
        kinematicAccuracy: kinematicResult.metrics?.positionAccuracy || 1,
        sensorAccuracy: sensorResult.metrics?.sensorAccuracy || 1,
        timingAccuracy: timingResult.metrics?.timingAccuracy || 1,
        overallScore: overallScore
      },
      details: {
        kinematic: kinematicResult,
        sensor: sensorResult,
        timing: timingResult
      }
    };
  }

  /**
   * Calculates an overall transfer score based on validation results
   * @param {object} kinematicResult - Kinematic validation result
   * @param {object} sensorResult - Sensor validation result
   * @param {object} timingResult - Timing validation result
   * @returns {number} Overall transfer score (0-1)
   */
  calculateTransferScore(kinematicResult, sensorResult, timingResult) {
    // Weight different aspects of the transfer
    const kinematicWeight = 0.4;
    const sensorWeight = 0.4;
    const timingWeight = 0.2;

    const kinematicScore = kinematicResult.isValid ?
      (kinematicResult.metrics?.positionAccuracy || 0.9) : 0;
    const sensorScore = sensorResult.isValid ?
      (sensorResult.metrics?.sensorAccuracy || 0.9) : 0;
    const timingScore = timingResult.isValid ?
      (timingResult.metrics?.timingAccuracy || 0.9) : 0;

    return kinematicWeight * kinematicScore +
           sensorWeight * sensorScore +
           timingWeight * timingScore;
  }

  /**
   * Generates a transfer validation report
   * @param {object} validationResult - Result from validateCompleteTransfer
   * @param {string} outputPath - Path to save the report
   * @returns {boolean} Success status
   */
  generateValidationReport(validationResult, outputPath) {
    const report = {
      timestamp: new Date().toISOString(),
      isValid: validationResult.isValid,
      overallScore: validationResult.metrics?.overallScore,
      summary: {
        errors: validationResult.errors.length,
        warnings: validationResult.warnings.length,
        suggestions: validationResult.suggestions.length
      },
      metrics: validationResult.metrics,
      details: validationResult.details,
      recommendations: this.generateRecommendations(validationResult)
    };

    try {
      fs.writeFileSync(outputPath, JSON.stringify(report, null, 2));
      return true;
    } catch (error) {
      console.error(`Failed to write validation report: ${error.message}`);
      return false;
    }
  }

  /**
   * Generates recommendations based on validation results
   * @param {object} validationResult - Validation result
   * @returns {array} List of recommendations
   */
  generateRecommendations(validationResult) {
    const recommendations = [];

    if (!validationResult.isValid) {
      recommendations.push("Address critical errors before attempting hardware transfer.");
    }

    if (validationResult.warnings.length > 0) {
      recommendations.push(`Review ${validationResult.warnings.length} warnings that may affect performance.`);
    }

    if (validationResult.metrics?.overallScore < 0.8) {
      recommendations.push("Overall transfer score is low. Consider additional simulation refinement.");
    }

    if (validationResult.metrics?.kinematicAccuracy < 0.9) {
      recommendations.push("Kinematic accuracy is low. Verify robot physical parameters in simulation.");
    }

    if (validationResult.metrics?.sensorAccuracy < 0.85) {
      recommendations.push("Sensor accuracy is low. Ensure sensor parameters match hardware specifications.");
    }

    if (validationResult.suggestions.length > 0) {
      recommendations.push("Implement the suggested improvements from validation results.");
    }

    return recommendations;
  }
}

module.exports = new SimulationTransferValidationService();