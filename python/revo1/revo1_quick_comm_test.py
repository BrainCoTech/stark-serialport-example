"""
Revo1 communication frequency test - quick demonstration example

This is a simplified version of the test example for quickly verifying the communication performance of Revo1.
"""

import asyncio
import time
import statistics
from revo1_utils import *

async def quick_frequency_test(duration=5.0, target_hz=50, test_type="read"):
    """
    Quick frequency test

    Args:
        duration: Test duration (seconds)
        target_hz: Target frequency (Hz)
        test_type: Test type "read"(status read) or "write"(position set) or "mixed"(mixed)
    """
    test_name_map = {
        "read": "get_motor_status status read",
        "write": "set_finger_positions position set",
        "mixed": "mixed function (read + set)"
    }

    print(f"🚀 Start {duration} seconds of {test_name_map.get(test_type, test_type)} frequency test")
    print(f"🎯 Target frequency: {target_hz} Hz")

    # Connect device
    try:
        client, slave_id = await open_modbus_revo1()
        logger.info(f"✅ Successfully connected device, slave ID: {slave_id}")
    except Exception as e:
        logger.error(f"❌ Connection failed: {e}")
        return

    # Test data recording
    results = []
    start_time = time.perf_counter()
    test_count = 0
    successful_count = 0

    # Position sequence (for write test)
    position_sequences = [
        [0, 0, 0, 0, 0, 0],            # Open
        [30, 30, 50, 50, 50, 50],      # Half grip
        [60, 60, 100, 100, 100, 100],  # Grip
    ]

    try:
        while (time.perf_counter() - start_time) < duration:
            test_start = time.perf_counter()

            try:
                positions = None  # Initialize variable
                if test_type == "read":
                    # Status read test
                    status = await client.get_motor_status(slave_id)
                elif test_type == "write":
                    # Position set test
                    positions = position_sequences[test_count % len(position_sequences)]
                    await client.set_finger_positions(slave_id, positions)
                elif test_type == "mixed":
                    # Mixed test: mainly read, occasionally set
                    if test_count % 5 == 0:
                        positions = position_sequences[test_count // 5 % len(position_sequences)]
                        await client.set_finger_positions(slave_id, positions)
                    else:
                        status = await client.get_motor_status(slave_id)
                else:
                    # Default read test
                    status = await client.get_motor_status(slave_id)

                elapsed_ms = (time.perf_counter() - test_start) * 1000
                results.append(elapsed_ms)
                successful_count += 1

                if test_count % 50 == 0:
                    action_desc = ""
                    if test_type == "write" and positions:
                        action_desc = f" - Position: {positions}"
                    elif test_type == "mixed":
                        action_desc = f" - {'Position set' if test_count % 5 == 0 else 'Status read'}"
                    print(f"  {test_count:3d}th test - Delay: {elapsed_ms:6.2f}ms{action_desc}")

            except Exception as e:
                print(f"  Test failed: {e}")

            test_count += 1

            # Control frequency
            sleep_time = 1.0 / target_hz - (time.perf_counter() - test_start)
            if sleep_time > 0:
                await asyncio.sleep(sleep_time)

        total_duration = time.perf_counter() - start_time

        # Calculate statistics
        if results:
            avg_latency = statistics.mean(results)
            max_latency = max(results)
            min_latency = min(results)
            std_dev = statistics.stdev(results) if len(results) > 1 else 0
            actual_frequency = test_count / total_duration
            success_rate = successful_count / test_count * 100

            print(f"\n{'='*50}")
            print(f"📊 Test result statistics")
            print(f"{'='*50}")
            print(f"Total test count:     {test_count}")
            print(f"Success count:       {successful_count}")
            print(f"Success rate:         {success_rate:.1f}%")
            print(f"Average delay:       {avg_latency:.2f} ms")
            print(f"Minimum delay:       {min_latency:.2f} ms")
            print(f"Maximum delay:       {max_latency:.2f} ms")
            print(f"Delay standard deviation:     {std_dev:.2f} ms")
            print(f"Actual frequency:       {actual_frequency:.1f} Hz")
            print(f"Target frequency:       {target_hz} Hz")
            print(f"Frequency achievement rate:     {(actual_frequency/target_hz*100):.1f}%")
            print(f"Test duration:       {total_duration:.1f} seconds")

            # Simple performance evaluation
            print(f"\n🔍 Performance evaluation:")
            if avg_latency < 10:
                print("✅ Communication delay excellent (< 10ms)")
            elif avg_latency < 20:
                print("🟡 Communication delay good (< 20ms)")
            else:
                print("🔴 Communication delay high (>= 20ms)")

            if success_rate >= 99:
                print("✅ Communication stability excellent (>= 99%)")
            elif success_rate >= 95:
                print("🟡 Communication stability good (>= 95%)")
            else:
                print("🔴 Communication stability needs improvement (< 95%)")

            if actual_frequency >= target_hz * 0.9:
                print("✅ Frequency achievement rate excellent (>= 90%)")
            elif actual_frequency >= target_hz * 0.8:
                print("🟡 Frequency achievement rate good (>= 80%)")
            else:
                print("🔴 Frequency achievement rate low (< 80%)")

        else:
            print("❌ No successful test results")

    except KeyboardInterrupt:
        print("\n⚠️  Test interrupted by user")

    finally:
        libstark.modbus_close(client)
        logger.info("🔌 Connection closed")

async def main():
    """Main function: quick test demonstration"""
    print("🤖 Revo1 communication frequency quick test")
    print("="*50)

    # Run different types and frequencies of tests
    test_configs = [
        # (duration, frequency, type, description)
        (5.0, 50, "read", "Status read test"),
        (5.0, 20, "write", "Position set test"),
        (5.0, 40, "mixed", "Mixed function test"),
    ]

    for duration, frequency, test_type, description in test_configs:
        print(f"\n📋 {description}")
        await quick_frequency_test(duration, frequency, test_type)
        print("\n" + "-"*50)
        await asyncio.sleep(1)  # Test interval

    print("\n🎉 All tests completed!")
    print("\n💡 Test summary:")
    print("   • Status read (get_motor_status): Used to monitor finger position and status in real-time")
    print("   • Position set (set_finger_positions): Used to control finger movement to target position")
    print("   • Mixed function: Simulates read+control operations in actual applications")

if __name__ == "__main__":
    asyncio.run(main())
