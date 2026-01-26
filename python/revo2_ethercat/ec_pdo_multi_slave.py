import asyncio
import time

from ec_utils import logger, libstark, setup_shutdown_event


async def control_slave(ctx: libstark.PyDeviceContext, slave_pos: int, is_touch: bool, is_pressure: bool):
    """
    Per-slave control & read task.
    - 周期性做：单指控制 → 多指控制 → 读取马达状态 (+ 触觉)
    - 注意：不要在持有 ctx 锁时长时间 sleep，避免多从站互相“饿死”
    """
    durations = [200] * 6
    finger_id = libstark.FingerId.Middle

    while True:
        # 单指：抬起
        start = time.perf_counter()
        await ctx.set_finger_position_with_millis(slave_pos, finger_id, 500, 300)
        elapsed = (time.perf_counter() - start) * 1000
        logger.info(f"[slave {slave_pos}] set_finger_position_with_millis(up), elapsed: {elapsed:.2f} ms")
        await asyncio.sleep(1.0)

        # 单指：放下
        start = time.perf_counter()
        await ctx.set_finger_position_with_millis(slave_pos, finger_id, 0, 300)
        elapsed = (time.perf_counter() - start) * 1000
        logger.info(f"[slave {slave_pos}] set_finger_position_with_millis(down), elapsed: {elapsed:.2f} ms")
        await asyncio.sleep(1.0)

        # 多指：握拳
        positions = [500, 500, 1000, 1000, 1000, 1000]
        start = time.perf_counter()
        await ctx.set_finger_positions_and_durations(slave_pos, positions, durations)
        elapsed = (time.perf_counter() - start) * 1000
        logger.info(f"[slave {slave_pos}] set_finger_positions_and_durations(fist), elapsed: {elapsed:.2f} ms")
        await asyncio.sleep(1.0)

        # 多指：张开
        positions = [0, 0, 0, 0, 0, 0]
        start = time.perf_counter()
        await ctx.set_finger_positions_and_durations(slave_pos, positions, durations)
        elapsed = (time.perf_counter() - start) * 1000
        logger.info(f"[slave {slave_pos}] set_finger_positions_and_durations(open), elapsed: {elapsed:.2f} ms")

        # 读取马达状态 + 触觉（仅少量次数，演示用）
        for i in range(2):
            try:
                logger.info(f"[slave {slave_pos}] get_motor_status[{i}] begin")
                start = time.perf_counter()
                status: libstark.MotorStatusData = await ctx.get_motor_status(slave_pos)
                elapsed = (time.perf_counter() - start) * 1000
                logger.info(
                    f"[slave {slave_pos}] get_motor_status[{i}] done, elapsed: {elapsed:.2f} ms, "
                    f"positions={status.positions}, currents={status.currents}"
                )

                if is_touch:
                    if is_pressure:
                        # 压力触觉（modulus）
                        logger.debug(f"[slave {slave_pos}] get_modulus_touch_summary")
                        start = time.perf_counter()
                        summary = await ctx.get_modulus_touch_summary(slave_pos)
                        elapsed = (time.perf_counter() - start) * 1000
                        logger.info(
                            f"[slave {slave_pos}] get_modulus_touch_summary, elapsed: {elapsed:.2f} ms, summary={summary}"
                        )
                    else:
                        # 电容触觉
                        logger.debug(f"[slave {slave_pos}] get_touch_sensor_status")
                        start = time.perf_counter()
                        touch_items = await ctx.get_touch_sensor_status(slave_pos)
                        elapsed = (time.perf_counter() - start) * 1000
                        logger.info(
                            f"[slave {slave_pos}] get_touch_sensor_status, elapsed: {elapsed:.2f} ms, count={len(touch_items)}"
                        )
                await asyncio.sleep(0.1)
            except Exception as e:
                logger.error(f"[slave {slave_pos}] read error: {e}")
                await asyncio.sleep(0.5)


async def main():
    shutdown_event = setup_shutdown_event(logger)

    master_pos = 0
    # 也可以用: ctx = libstark.ethercat_open_master(master_pos)
    ctx = libstark.PyDeviceContext.open_ethercat_master(master_pos)

    # 多从站位置（根据实际 ethercat slaves 输出来调整）
    slave_positions = [1, 2]

    # 为每个从站做 SDO 配置，并记录各自的触觉类型
    from collections import defaultdict

    slave_cfg: dict[int, dict] = defaultdict(dict)

    for slave_pos in slave_positions:
        logger.info(f"Setting up SDO for slave {slave_pos}")
        await ctx.ec_setup_sdo(slave_pos)

        info: libstark.DeviceInfo = await ctx.get_device_info(slave_pos)
        is_touch = info.is_touch()
        is_pressure = ctx.is_touch_pressure(slave_pos)  # 此时 ctx.touch_vendor 针对当前 slave_pos
        slave_cfg[slave_pos]["info"] = info
        slave_cfg[slave_pos]["is_touch"] = is_touch
        slave_cfg[slave_pos]["is_pressure"] = is_pressure

        logger.info(
            f"Slave {slave_pos} DeviceInfo: {info.description}, "
            f"is_touch={is_touch}, is_pressure={is_pressure}"
        )

    # 预留主站并启动 PDO 循环（一次性传入所有从站位置）
    await ctx.ec_reserve_master()
    logger.info("EtherCAT master reserved")

    await ctx.ec_start_loop(slave_positions, 0, 1_000_000, 0, 0, 0)
    logger.info(f"EtherCAT PDO loop started with slaves: {slave_positions}")

    # 为每个从站启动独立的控制任务
    tasks: list[asyncio.Task] = []
    for slave_pos in slave_positions:
        is_touch = bool(slave_cfg[slave_pos].get("is_touch"))
        is_pressure = bool(slave_cfg[slave_pos].get("is_pressure"))
        task = asyncio.create_task(control_slave(ctx, slave_pos, is_touch, is_pressure))
        tasks.append(task)

    # 演示运行一段时间，期间可以 Ctrl+C 退出
    await asyncio.sleep(60)

    # 停止所有任务和 EtherCAT 循环
    for t in tasks:
        t.cancel()
    await ctx.ec_stop_loop()
    logger.info("EtherCAT loop stopped")
    await ctx.close()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("User interrupted")
        sys.exit(0)
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        sys.exit(1)


