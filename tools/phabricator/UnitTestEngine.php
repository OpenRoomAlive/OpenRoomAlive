<?php


/**
 * Unit test engine for CMake and GTest.
 */
final class UnitTestEngine extends ArcanistUnitTestEngine {

  /**
   * Arcanist should not print test output.
   */
  public function shouldEchoTestResults() {
    return false;
  }

  /**
   * Runs the test suite.
   */
  public function run() {
    $results = array();

    $command  = '(mkdir -p build && cd build && cmake ..)';
    $command .= '&& make -C build all';
    $command .= '&& make -C build test';

    // Execute the test command & time it.
    $timeStart = microtime(true);
    {
      $future = new ExecFuture($command);
      do {
        $future->read();
        sleep(0.5);
      } while (!$future->isReady());
      list($error, $stdout, $stderr) = $future->resolve();
    }
    $timeEnd = microtime(true);

    // Create a unit test result structure.
    $result = new ArcanistUnitTestResult();
    $result->setNamespace('DerpVision');
    $result->setName('Core');
    $result->setDuration($timeEnd - $timeStart);

    if ($error == 0) {
      $result->setResult(ArcanistUnitTestResult::RESULT_PASS);
    } else {
      $result->setResult(ArcanistUnitTestResult::RESULT_FAIL);
      $result->setUserData($stdout . $stderr);
    }

    $results[] = $result;
    return $results;
  }
}
