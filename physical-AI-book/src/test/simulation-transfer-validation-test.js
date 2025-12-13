/**
 * Test suite for Simulation-to-Hardware Transfer Validation Service
 * Tests validation of simulation behaviors matching hardware behaviors
 */

const SimulationTransferValidationService = require('../services/simulation-transfer-validation-service');

function runTests() {
  console.log('🧪 Running Simulation-to-Hardware Transfer Validation Tests...\n');

  let testCount = 0;
  let passedCount = 0;

  // Test 1: Valid kinematic transfer
  testCount++;
  console.log(`Test ${testCount}: Valid kinematic transfer`);
  const validSimKinematics = {
    wheelRadius: 0.033,
    wheelSeparation: 0.16,
    linearVelocity: 0.3,
    angularVelocity: 0.5,
    trajectory: {
      positions: [
        {x: 0, y: 0}, {x: 0.1, y: 0.05}, {x: 0.2, y: 0.1}
      ]
    }
  };

  const validHwKinematics = {
    linearVelocity: 0.31,
    angularVelocity: 0.49,
    trajectory: {
      positions: [
        {x: 0, y: 0}, {x: 0.11, y: 0.04}, {x: 0.21, y: 0.11}
      ]
    }
  };

  const kinematicResult = SimulationTransferValidationService.validateKinematicTransfer(
    validSimKinematics, validHwKinematics, 'jetbot'
  );
  console.log(`  Result: ${kinematicResult.isValid ? '✅ PASS' : '❌ FAIL'}`);
  console.log(`  Errors: ${kinematicResult.errors.length}, Warnings: ${kinematicResult.warnings.length}`);
  if (kinematicResult.isValid) passedCount++;
  console.log('');

  // Test 2: Invalid kinematic transfer (large velocity mismatch)
  testCount++;
  console.log(`Test ${testCount}: Invalid kinematic transfer (large velocity mismatch)`);
  const invalidSimKinematics = {
    linearVelocity: 0.5,
    angularVelocity: 1.0
  };

  const invalidHwKinematics = {
    linearVelocity: 0.1, // Large mismatch
    angularVelocity: 0.1
  };

  const invalidKinematicResult = SimulationTransferValidationService.validateKinematicTransfer(
    invalidSimKinematics, invalidHwKinematics, 'jetbot'
  );
  console.log(`  Result: ${invalidKinematicResult.errors.length > 0 ? '✅ PASS (detected errors)' : '❌ FAIL (should detect errors)'}`);
  console.log(`  Errors: ${invalidKinematicResult.errors.length}, Warnings: ${invalidKinematicResult.warnings.length}`);
  if (invalidKinematicResult.errors.length > 0) passedCount++;
  console.log('');

  // Test 3: Valid sensor transfer
  testCount++;
  console.log(`Test ${testCount}: Valid sensor transfer`);
  const validSimSensors = {
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
      updateRate: 100,
      noise: { accel: 0.01, gyro: 0.001 }
    }
  };

  const validHwSensors = {
    camera: {
      resolution: { width: 640, height: 480 },
      frameRate: 30,
      fov: 60
    },
    lidar: {
      range: { max: 5.2 },
      fov: 270,
      resolution: 720
    },
    imu: {
      updateRate: 100,
      noise: { accel: 0.012, gyro: 0.0011 }
    }
  };

  const sensorResult = SimulationTransferValidationService.validateSensorTransfer(
    validSimSensors, validHwSensors
  );
  console.log(`  Result: ${sensorResult.isValid ? '✅ PASS' : '❌ FAIL'}`);
  console.log(`  Errors: ${sensorResult.errors.length}, Warnings: ${sensorResult.warnings.length}`);
  if (sensorResult.isValid) passedCount++;
  console.log('');

  // Test 4: Invalid sensor transfer (LiDAR range mismatch)
  testCount++;
  console.log(`Test ${testCount}: Invalid sensor transfer (LiDAR range mismatch)`);
  const invalidSimSensors = {
    lidar: {
      range: { max: 5.0 }
    }
  };

  const invalidHwSensors = {
    lidar: {
      range: { max: 2.0 } // Large mismatch
    }
  };

  const invalidSensorResult = SimulationTransferValidationService.validateSensorTransfer(
    invalidSimSensors, invalidHwSensors
  );
  console.log(`  Result: ${invalidSensorResult.errors.length > 0 ? '✅ PASS (detected errors)' : '❌ FAIL (should detect errors)'}`);
  console.log(`  Errors: ${invalidSensorResult.errors.length}, Warnings: ${invalidSensorResult.warnings.length}`);
  if (invalidSensorResult.errors.length > 0) passedCount++;
  console.log('');

  // Test 5: Valid timing transfer
  testCount++;
  console.log(`Test ${testCount}: Valid timing transfer`);
  const validSimTiming = {
    controlLoopRate: 50,
    computationTime: 10,
    latency: 5
  };

  const validHwTiming = {
    controlLoopRate: 48,
    computationTime: 12,
    latency: 6
  };

  const timingResult = SimulationTransferValidationService.validateTimingTransfer(
    validSimTiming, validHwTiming
  );
  console.log(`  Result: ${timingResult.isValid ? '✅ PASS' : '❌ FAIL'}`);
  console.log(`  Errors: ${timingResult.errors.length}, Warnings: ${timingResult.warnings.length}`);
  if (timingResult.isValid) passedCount++;
  console.log('');

  // Test 6: Invalid timing transfer (control loop mismatch)
  testCount++;
  console.log(`Test ${testCount}: Invalid timing transfer (control loop mismatch)`);
  const invalidSimTiming = {
    controlLoopRate: 100
  };

  const invalidHwTiming = {
    controlLoopRate: 20 // Large mismatch
  };

  const invalidTimingResult = SimulationTransferValidationService.validateTimingTransfer(
    invalidSimTiming, invalidHwTiming
  );
  console.log(`  Result: ${invalidTimingResult.errors.length > 0 ? '✅ PASS (detected errors)' : '❌ FAIL (should detect errors)'}`);
  console.log(`  Errors: ${invalidTimingResult.errors.length}, Warnings: ${invalidTimingResult.warnings.length}`);
  if (invalidTimingResult.errors.length > 0) passedCount++;
  console.log('');

  // Test 7: Complete transfer validation
  testCount++;
  console.log(`Test ${testCount}: Complete transfer validation`);
  const completeSimData = {
    kinematics: validSimKinematics,
    sensors: validSimSensors,
    timing: validSimTiming
  };

  const completeHwData = {
    kinematics: validHwKinematics,
    sensors: validHwSensors,
    timing: validHwTiming
  };

  const completeResult = SimulationTransferValidationService.validateCompleteTransfer(
    completeSimData, completeHwData, 'jetbot'
  );
  console.log(`  Result: ${completeResult.isValid ? '✅ PASS' : '❌ FAIL'}`);
  console.log(`  Errors: ${completeResult.errors.length}, Warnings: ${completeResult.warnings.length}`);
  console.log(`  Overall Score: ${(completeResult.metrics?.overallScore * 100).toFixed(2)}%`);
  if (completeResult.isValid) passedCount++;
  console.log('');

  // Test 8: Invalid complete transfer
  testCount++;
  console.log(`Test ${testCount}: Invalid complete transfer`);
  const invalidCompleteSimData = {
    kinematics: invalidSimKinematics,
    sensors: invalidSimSensors,
    timing: invalidSimTiming
  };

  const invalidCompleteHwData = {
    kinematics: invalidHwKinematics,
    sensors: invalidHwSensors,
    timing: invalidHwTiming
  };

  const invalidCompleteResult = SimulationTransferValidationService.validateCompleteTransfer(
    invalidCompleteSimData, invalidCompleteHwData, 'jetbot'
  );
  console.log(`  Result: ${invalidCompleteResult.errors.length > 0 ? '✅ PASS (detected errors)' : '❌ FAIL (should detect errors)'}`);
  console.log(`  Errors: ${invalidCompleteResult.errors.length}, Warnings: ${invalidCompleteResult.warnings.length}`);
  console.log(`  Overall Score: ${(invalidCompleteResult.metrics?.overallScore * 100).toFixed(2)}%`);
  if (invalidCompleteResult.errors.length > 0) passedCount++;
  console.log('');

  // Test 9: Generate validation report
  testCount++;
  console.log(`Test ${testCount}: Generate validation report`);
  try {
    const reportPath = './temp_validation_report.json';
    const reportSuccess = SimulationTransferValidationService.generateValidationReport(
      completeResult, reportPath
    );
    console.log(`  Result: ${reportSuccess ? '✅ PASS' : '❌ FAIL'}`);
    if (reportSuccess) {
      // Clean up
      const fs = require('fs');
      if (fs.existsSync(reportPath)) {
        fs.unlinkSync(reportPath);
      }
      passedCount++;
    }
  } catch (error) {
    console.log(`  Result: ❌ FAIL (error: ${error.message})`);
  }
  console.log('');

  // Test 10: Unknown robot type
  testCount++;
  console.log(`Test ${testCount}: Unknown robot type`);
  const unknownRobotResult = SimulationTransferValidationService.validateKinematicTransfer(
    {}, {}, 'unknown_robot'
  );
  console.log(`  Result: ${unknownRobotResult.errors.length > 0 ? '✅ PASS (detected error)' : '❌ FAIL (should detect error)'}`);
  if (unknownRobotResult.errors.length > 0) passedCount++;
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